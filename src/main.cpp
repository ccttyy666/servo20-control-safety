// 20-axis servo/safety learning simulator, not safety-certified hardware code.
// Linux/WSL: g++ -std=c++17 -O2 -Wall -Wextra src/main.cpp -o servo20
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr int N = 20;
constexpr double DT = 0.001;                 // simulation/control step: 1 ms
constexpr double MOVE_START = 0.25;
constexpr double MOVE_DURATION = 3.0;
constexpr double TOTAL_TIME = 4.5;
constexpr int JAM_JOINT = 7;                // zero-based -> joint 08
constexpr int DRIVE_HB_JOINT = 12;          // zero-based -> joint 13
constexpr double FAULT_AT = 1.45;
constexpr double HB_RESUME_AT = 1.75;
constexpr double HB_TIMEOUT = 0.050;       // 50 ms (demo threshold)
constexpr double CMD_TIMEOUT = 0.045;      // 45 ms, drive local watchdog
constexpr double FOLLOW_ERROR_LIMIT = 0.065; // rad (demo threshold)
constexpr int FOLLOW_ERROR_TICKS = 60;       // 60 ms debounce
constexpr double SAFESTOP_DEADLINE = 0.8;  // seconds, simulated brake timeout
constexpr double ARRIVAL_POS_TOL = 0.006; // rad
constexpr double ARRIVAL_VEL_TOL = 0.014; // rad/s
constexpr int ARRIVAL_HOLD_TICKS = 40;

using WallClock = std::chrono::steady_clock;
const char* state_name(int s) {
    switch(s) {
        case 0: return "RUNNING";
        case 1: return "SAFE_STOP";
        case 2: return "STOPPED";
        case 3: return "ESTOP";
        default: return "UNKNOWN";
    }
}
struct Reference {double q{0}, v{0}, a{0};};
struct JointConfig {
    double inertia{}, damping{}, coulomb{}, max_torque{}, max_slew{},
           max_velocity{}, max_position{};
};
JointConfig make_config(int i) {
    return {
        0.012 + 0.001 * i,             // kg*m^2
        0.027 + 0.004 * (i % 5),      // N*m*s/rad
        0.0025 + 0.0005 * (i % 4),   // smooth Coulomb friction (N*m)
        0.70 + 0.055 * (i % 7),      // N*m
        20.0 + 2.0 * (i % 4),        // N*m/s
        1.0 + 0.06 * (i % 5),        // rad/s
        1.7                           // +/- rad
    };
}
double final_target(int i) {
    double size = 0.20 + 0.023 * i;
    return (i % 2 == 0 ? 1.0 : -1.0) * size;
}
Reference ref_at(int i, double time) {
    const double delta = final_target(i);
    if (time <= MOVE_START) return {};
    if (time >= MOVE_START + MOVE_DURATION) return {delta, 0.0, 0.0};
    double r = (time - MOVE_START) / MOVE_DURATION;
    double s = 10*r*r*r - 15*r*r*r*r + 6*r*r*r*r*r;
    double ds = 30*r*r - 60*r*r*r + 30*r*r*r*r;
    double dds = 60*r - 180*r*r + 120*r*r*r;
    return {delta*s, delta*ds/MOVE_DURATION,
            delta*dds/(MOVE_DURATION*MOVE_DURATION)};
}

// PI(D) with desired-velocity derivative feedback; output torque.
// Called at a fixed simulated dt. All inputs must have been sanity-checked.
struct PID {
    double kp{},ki{},kd{}, integral{0};
    double update(const JointConfig& cfg, const Reference& r,
                  double q, double v) {
        const double error = r.q - q;
        const double ff = cfg.inertia*r.a + cfg.damping*r.v
                        + cfg.coulomb*std::tanh(r.v/0.025);
        const double pd_ff = kp*error + kd*(r.v-v) + ff;
        const double candidate = std::clamp(integral + error*DT,-0.15,0.15);
        const double trial = pd_ff + ki*candidate;
        const bool further_saturation =
            (trial > cfg.max_torque && error > 0.0) ||
            (trial < -cfg.max_torque && error < 0.0);
        if (!further_saturation) integral = candidate;
        return std::clamp(pd_ff + ki*integral,
                          -cfg.max_torque,cfg.max_torque);
    }
    void reset() { integral=0; }
};

// Fixed-size pending-command slots: no per-step heap allocation.
struct Packet {int deliver_tick{-1}; int sequence{-1}; double torque{0};};
struct DelayLine {
    std::array<Packet,32> slots{};
    int last_sequence{-1};
    unsigned lost{0};
    void push(int delivery, int sequence, double torque) {
        for (auto& p: slots) {
            if (p.deliver_tick < 0) {p={delivery,sequence,torque};return;}
        }
        ++lost;  // must be treated as a fault in a production system
    }
    bool consume(int tick, double& torque) {
        bool changed = false;
        int best_seq = last_sequence;
        double best_torque = torque;
        for (auto& p: slots) {
            if (p.deliver_tick >= 0 && p.deliver_tick <= tick) {
                if (p.sequence > best_seq) {
                    best_seq = p.sequence;
                    best_torque = p.torque;
                    changed = true;
                }
                p.deliver_tick = -1;
            }
        }
        if (changed) {
            last_sequence = best_seq;
            torque = best_torque;
        }
        return changed;
    }
};

struct Joint {
    JointConfig cfg{};
    PID pid{};
    DelayLine pipe{};
    double q{0}, v{0}, applied_torque{0}, last_command{0};
    double last_command_time{-1};
    double last_drive_hb{0};
    int arrival_hold_ticks{0};
    double arrival_time{std::numeric_limits<double>::quiet_NaN()};
    int error_ticks{0};
    bool jammed{false}, local_watchdog{false};

    void simulate(double requested_torque, bool safe_brake) {
        if (safe_brake) requested_torque = -std::clamp(7.5*v,
                                 -cfg.max_torque,cfg.max_torque);
        requested_torque = std::clamp(requested_torque,
                                      -cfg.max_torque,cfg.max_torque);
        double delta = std::clamp(requested_torque-applied_torque,
                                  -cfg.max_slew*DT,cfg.max_slew*DT);
        applied_torque = std::clamp(applied_torque+delta,
                                   -cfg.max_torque,cfg.max_torque);
        if (jammed) {
            // A mechanical lock prevents motion but not motor torque buildup.
            // We DO NOT simulate detailed gear stress / electrical current.
            v = 0;
            return;
        }
        const double friction = cfg.coulomb*std::tanh(v/0.025);
        const double acc=(applied_torque-cfg.damping*v-friction)/cfg.inertia;
        v += acc*DT;
        q += v*DT;
        // Deliberately do not silently clamp q/v to physical limits:
        // supervisor must detect limit violations.
    }
};

struct Snapshot {
    double t{}, wakeup_us{std::numeric_limits<double>::quiet_NaN()},
           period_error_us{std::numeric_limits<double>::quiet_NaN()},
           hb_age_ms{}, max_error{}, max_speed{};
    int state{};
    std::array<Reference,N> ref{};
    std::array<double,N> q{},v{},tau{},error{};
};
struct Event {double t{};std::string type;};

std::string csv_safe(std::string s) {
    for(char& c:s) if(c==',' || c=='\n')c=';';
    return s;
}
void save_csv(const std::filesystem::path& path,
              const std::vector<Snapshot>& logs) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot write CSV: " + path.string());
    f << "t_s,state,wakeup_lateness_us,period_error_us,controller_hb_age_ms,max_abs_tracking_err_rad,max_abs_vel_rad_s";
    for(int i=0;i<N;++i) {
        const std::string id=std::to_string(i+1);
        f<<",q_ref_"<<id<<",q_"<<id<<",error_"<<id
         <<",v_"<<id<<",torque_"<<id;
    }
    f<<'\n'<<std::fixed<<std::setprecision(7);
    for(const auto& row:logs) {
        f<<row.t<<','<<state_name(row.state)<<',';
        if(std::isfinite(row.wakeup_us))f<<row.wakeup_us;
        f<<',';
        if(std::isfinite(row.period_error_us))f<<row.period_error_us;
        f<<','<<row.hb_age_ms<<','<<row.max_error<<','<<row.max_speed;
        for(int i=0;i<N;++i)
            f<<','<<row.ref[i].q<<','<<row.q[i]<<','<<row.error[i]
             <<','<<row.v[i]<<','<<row.tau[i];
        f<<'\n';
    }
}
void save_events(const std::filesystem::path& path,const std::vector<Event>& events) {
    std::ofstream f(path);
    f<<"time_s,event\n"<<std::fixed<<std::setprecision(4);
    for(const auto& e:events)f<<e.t<<','<<csv_safe(e.type)<<'\n';
}
double percentile(std::vector<double> data, double p) {
    if(data.empty())return std::numeric_limits<double>::quiet_NaN();
    std::sort(data.begin(),data.end());
    return data[static_cast<size_t>(std::lround((data.size()-1)*p))];
}

void run(const std::string& scenario, bool realtime,
         const std::filesystem::path& outdir) {
    std::array<Joint,N> joints{};
    for(int i=0;i<N;++i) {
        joints[i].cfg=make_config(i);
        const double wn=22.0,zeta=0.90;
        joints[i].pid.kp=joints[i].cfg.inertia*wn*wn;
        joints[i].pid.kd=std::max(0.0,2*zeta*wn*joints[i].cfg.inertia-joints[i].cfg.damping);
        joints[i].pid.ki=1.5;
    }
    constexpr int steps=static_cast<int>(TOTAL_TIME/DT);
    std::vector<Snapshot> logs;
    logs.reserve(steps+1);  // allocation outside periodic control loop
    std::vector<Event> events;
    events.reserve(20);
    int state=0;
    double stop_started=std::numeric_limits<double>::quiet_NaN();
    double last_controller_heartbeat=0;
    int stable_stop_ticks=0;
    int first_trigger_tick=-1;
    std::string reason="none";

    const auto epoch=WallClock::now()+std::chrono::milliseconds(20);
    auto prev_wake=epoch-std::chrono::microseconds(1000);
    for(int tick=0;tick<=steps;++tick) {
        const double t=tick*DT;
        Snapshot log{};
        log.t=t;
        if(realtime) {
            const auto scheduled=epoch+std::chrono::microseconds(1000LL*tick);
            std::this_thread::sleep_until(scheduled);
            const auto wake=WallClock::now();
            log.wakeup_us=std::chrono::duration<double,std::micro>(wake-scheduled).count();
            if(tick>0)
                log.period_error_us=std::chrono::duration<double,std::micro>(wake-prev_wake).count()-1000.0;
            prev_wake=wake;
        }
        const bool ctrl_paused = scenario=="controller_heartbeat" &&
                                  t>=FAULT_AT && t<HB_RESUME_AT;
        const bool drive_hb_paused = scenario=="drive_heartbeat" &&
                                   t>=FAULT_AT && t<HB_RESUME_AT;
        if(tick%10==0 && !ctrl_paused)last_controller_heartbeat=t;
        for(int i=0;i<N;++i) {
            if(tick%10==0 && !(drive_hb_paused && i==DRIVE_HB_JOINT))
                joints[i].last_drive_hb=t;
        }
        if(scenario=="jam" && tick==static_cast<int>(FAULT_AT/DT)) {
            joints[JAM_JOINT].jammed=true;
            events.push_back({t,"mechanical_jam_injected joint=8 (simulation only)"});
        }
        if(tick==static_cast<int>(FAULT_AT/DT)) {
            if(scenario=="controller_heartbeat")events.push_back({t,"controller_hang_injected"});
            if(scenario=="drive_heartbeat")events.push_back({t,"drive_heartbeat_loss_injected joint=13"});
        }
        // One shared time base: 20 trajectory references sampled at the same t.
        for(int i=0;i<N;++i) log.ref[i]=ref_at(i,t);
        if(state==0 && !ctrl_paused) {
            for(int i=0;i<N;++i) {
                auto& j=joints[i];
                const double torque=j.pid.update(j.cfg,log.ref[i],j.q,j.v);
                // Variable bus delay: base 1..4 ms, occasional +2 ms spike.
                const int delay_ticks=1+(i%4)+(((tick/150)%7==3)?2:0);
                j.pipe.push(tick+delay_ticks,tick,torque);
            }
        }
        for(int i=0;i<N;++i) {
            auto& j=joints[i];
            double received=j.last_command;
            const bool fresh=j.pipe.consume(tick,received);
            if(fresh){j.last_command=received;j.last_command_time=t;}
            j.local_watchdog = (t>0.070 &&
                (j.last_command_time<0 || t-j.last_command_time>CMD_TIMEOUT));
        }
        auto trigger=[&](const std::string& msg) {
            if(state!=0)return;
            state=1;
            first_trigger_tick=tick;
            stop_started=t;
            reason=msg;
            events.push_back({t,"SAFE_STOP: "+msg});
            for(auto& j:joints)j.pid.reset();
        };
        // Independent supervisory safety check (demo, NOT safety-rated).
        if(state==0) {
            if(t-last_controller_heartbeat>HB_TIMEOUT)
                trigger("controller_heartbeat_timeout");
            for(int i=0;i<N && state==0;++i) {
                auto& j=joints[i];
                if(t-j.last_drive_hb>HB_TIMEOUT)
                    trigger("drive_heartbeat_timeout joint="+std::to_string(i+1));
                else if(j.local_watchdog)
                    trigger("drive_command_watchdog_timeout joint="+std::to_string(i+1));
                else if(!std::isfinite(j.q) || !std::isfinite(j.v))
                    trigger("invalid_feedback joint="+std::to_string(i+1));
                else if(std::abs(j.q)>j.cfg.max_position ||
                        std::abs(j.v)>j.cfg.max_velocity)
                    trigger("joint_motion_limit joint="+std::to_string(i+1));
                else {
                    if(t>MOVE_START+0.15 &&
                       std::abs(log.ref[i].q-j.q)>FOLLOW_ERROR_LIMIT)
                        ++j.error_ticks;
                    else j.error_ticks=0;
                    if(j.error_ticks>=FOLLOW_ERROR_TICKS)
                        trigger("persistent_following_error joint="+std::to_string(i+1));
                }
            }
        }
        // If the central controller is frozen, drive-local watchdog or the
        // independent safety supervisor still initiate simulated braking.
        for(int i=0;i<N;++i) {
            auto& j=joints[i];
            const bool brake=(state!=0 || j.local_watchdog);
            j.simulate(j.last_command,brake);
        }
        double max_speed=0,max_error=0;
        for(int i=0;i<N;++i) {
            auto& j=joints[i];
            log.q[i]=j.q;
            log.v[i]=j.v;
            log.tau[i]=j.applied_torque;
            log.error[i]=log.ref[i].q-j.q;
            max_speed=std::max(max_speed,std::abs(j.v));
            max_error=std::max(max_error,std::abs(log.error[i]));
            // Arrival is a measured stable-window event, NOT just planned T.
            if(state==0 && t>=MOVE_START+MOVE_DURATION-0.30 &&
               !std::isfinite(j.arrival_time)) {
                if(std::abs(j.q-final_target(i))<ARRIVAL_POS_TOL &&
                   std::abs(j.v)<ARRIVAL_VEL_TOL)
                    ++j.arrival_hold_ticks;
                else j.arrival_hold_ticks=0;
                if(j.arrival_hold_ticks>=ARRIVAL_HOLD_TICKS)
                    j.arrival_time=t-(ARRIVAL_HOLD_TICKS-1)*DT;
            }
        }
        if(state==1) {
            if(max_speed<0.006) ++stable_stop_ticks;
            else stable_stop_ticks=0;
            if(stable_stop_ticks>=50) {
                state=2;
                events.push_back({t,"STOPPED: all_joint_velocity_below_0.006rad_s for_50ms"});
            } else if(t-stop_started>SAFESTOP_DEADLINE) {
                state=3;
                events.push_back({t,"ESTOP: simulated_brake_timeout - hardware_safety_required"});
            }
        }
        log.state=state;
        log.hb_age_ms=1000*(t-last_controller_heartbeat);
        log.max_error=max_error;
        log.max_speed=max_speed;
        logs.push_back(log);
    }
    std::filesystem::create_directories(outdir);
    save_csv(outdir/(scenario+".csv"),logs);
    save_events(outdir/(scenario+"_events.csv"),events);
    std::vector<double> arrival, lateness, period_err;
    for(auto& j:joints)
        if(std::isfinite(j.arrival_time)) arrival.push_back(j.arrival_time);
    for(const auto& l:logs) {
        if(std::isfinite(l.wakeup_us))lateness.push_back(l.wakeup_us);
        if(std::isfinite(l.period_error_us))period_err.push_back(std::abs(l.period_error_us));
    }
    std::ostringstream summary;
    summary<<std::fixed<<std::setprecision(5)
        <<"scenario="<<scenario<<"\n"
        <<"mode="<<(realtime?"wall_clock_sleep_until":"virtual_fast_simulation")<<"\n"
        <<"control_period_s="<<DT<<"\n"
        <<"joint_count="<<N<<"\n"
        <<"final_state="<<state_name(state)<<"\n"
        <<"stop_reason="<<reason<<"\n"
        <<"safety_trigger_s="<<(first_trigger_tick<0?-1:first_trigger_tick*DT)<<"\n"
        <<"stopped_s="<<(state==2 ? events.back().t : -1)<<"\n"
        <<"arrived_count="<<arrival.size()<<"/20\n"
        <<"arrival_spread_ms="<<(arrival.size()==N ?
             1000*(*std::max_element(arrival.begin(),arrival.end())-
             *std::min_element(arrival.begin(),arrival.end())):-1)<<"\n"
        <<"wakeup_lateness_p99_us="<<percentile(lateness,.99)<<"\n"
        <<"wakeup_lateness_max_us="<<(lateness.empty()?-1:*std::max_element(lateness.begin(),lateness.end()))<<"\n"
        <<"period_abs_error_p99_us="<<percentile(period_err,.99)<<"\n";
    for(int i=0;i<N;++i)summary<<"arrival_joint_"<<i+1<<"_s="<<joints[i].arrival_time<<"\n";
    std::ofstream out(outdir/(scenario+"_summary.txt"));out<<summary.str();
    std::cout<<summary.str()<<"\n";
}
} // namespace

int main(int argc,char** argv) {
    try {
        std::string scenario="nominal";
        bool realtime=false;
        std::filesystem::path outdir="results";
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--scenario" && i+1<argc)scenario=argv[++i];
            else if(arg=="--out" && i+1<argc)outdir=argv[++i];
            else if(arg=="--realtime")realtime=true;
            else if(arg=="--help") {
                std::cout<<"Usage: servo20 --scenario nominal|jam|controller_heartbeat|drive_heartbeat [--realtime] [--out DIR]\n";
                return 0;
            } else throw std::runtime_error("Unknown/missing argument: "+arg);
        }
        if(scenario!="nominal" && scenario!="jam" &&
           scenario!="controller_heartbeat" && scenario!="drive_heartbeat")
            throw std::runtime_error("Invalid scenario: "+scenario);
        run(scenario,realtime,outdir);
        return 0;
    }catch(const std::exception& e) {
        std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;
    }
}
