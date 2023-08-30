#include <iostream>
#include <iomanip>
#include <fstream>
#include <array>
#include <cmath>
#include <chrono>
#include <thread>

#include <AIToolbox/MDP/Algorithms/ValueIteration.hpp>
#include <AIToolbox/MDP/Policies/Policy.hpp>
#include <AIToolbox/MDP/IO.hpp>
#include <AIToolbox/MDP/SparseModel.hpp>

using StateType = std::array<int, 3>;

enum {
    RETRY_ATTEMPT = 0,
    HUMAN_ATTEMPT = 1,
    SUCCESS = 2
};


size_t A = 5;
enum {
    RETRY   = 0,
    HUMAN   = 1,
};

double getTransitionProbability( const StateType & s1, size_t action, const StateType & s2 ) {
    // Both the tiger and the antelope can only move by 1 cell max at each
    // timestep. Thus, if this is not the case, the transition is
    // impossible.
    if ( s1(HUMAN_ATTEMPT) > max_human_attempts ) return 0.0;

    if ( s1(RETRY_ATTEMPT) > max_retry_attempts ) return 0.0;

    // We check whether they were both in the same cell before.
    // In that case the game would have ended, and nothing would happen anymore.
    // We model this as a self-absorbing state, or a state that always transitions
    // to itself. This is valid no matter the action taken.
    if ( s1(SUCCESS) == 1) return 1.0;
    
    if ( s2(HUMAN_ATTEMPT) == 2 ) return 1.0;
    if ( s2(RETRY_ATTEMPT) == 3 ) return 1.0;

    if ( s1(RETRY_ATTEMPT) < s2(RETRY_ATTEMPT) && s1(HUMAN_ATTEMPT) < s2(HUMAN_ATTEMPT)) return 0;
    if ( s1(RETRY_ATTEMPT) > s2(RETRY_ATTEMPT) || s1(HUMAN_ATTEMPT) > s2(HUMAN_ATTEMPT)) return 0;

    // The tiger can move only in the direction specified by its action. If
    // it is not the case, the transition is impossible.
    if ( action == RETRY && (s1(HUMAN_ATTEMPT) < s2(HUMAN_ATTEMPT)) ) return 0.0;
    if ( action == HUMAN && (s1(RETRY_ATTEMPT) < s2(RETRY_ATTEMPT)) ) return 0.0;

    if ( action == RETRY && (s1(RETRY_ATTEMPT) < s2(RETRY_ATTEMPT)) ) return 0.9;
    if ( action == RETRY && (s2(SUCCESS) == 1 )) return 0.1;

    if ( action == HUMAN && (s1(HUMAN_ATTEMPT)) < s2(HUMAN_ATTEMPT) ) return 0.15;
    if ( action == HUMAN && (s2(SUCCESS) == 1)) return 0.85;

    // Else the probability of this transition is 1 / 4, still random but without
    // a possible antelope action.
    return 1.0;
}

double getReward( const StateType & s ) {
    if ( s[SUCCESS] == 1) return 10.0;
    if ( s[SUCCESS] == 0 && s[RETRY_ATTEMPT] > 0) return -2;
    if ( s[SUCCESS] == 0 && s[HUMAN_ATTEMPT] > 0) return -5;
    return 0.0;
}

class StateMachine {
    public:
        size_t getS() const { return 3 * 3 * 3; }
        size_t getA() const { return ::A; }
        double getDiscount() const { return ::discount; }

        double getTransitionProbability( size_t s, size_t a, size_t s1 ) const {
            return ::getTransitionProbability( decodeState( s ), a, decodeState( s1 ) );
        }

        double getExpectedReward( size_t, size_t, size_t s1 ) const {
            return ::getReward( decodeState( s1 ) );
        }

        // These two functions are needed to keep template code in the library
        // simple, but you don't need to implement them for the method we use
        // in this example.
        std::tuple<size_t, double> sampleSR(size_t,size_t) const;
        bool isTerminal(size_t) const;
};

bool StateMachine::isTerminal(size_t state){
    return state[SUCCESS] == 1 ? true:false;
}

void printCurrentTimeString() {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto tm = *std::localtime(&t);
    std::cout << std::put_time(&tm, "%T");
}

int main() {
    // Define a world or make your own modek here 
    StateMachine world;

    // This is optional, and should make solving the model almost instantaneous.
    // Unfortunately, since our main model is so big, the copying process
    // still takes a lot of time. But at least that would be a one-time cost!
    printCurrentTimeString(); std::cout << " - Constructing MDP...\n";
    AIToolbox::MDP::SparseModel model(world);

    // This is a method that solves MDPs completely. It has a couple of
    // parameters available.
    // The only non-optional parameter is the horizon of the solution; as in
    // how many steps should the solution look ahead in order to decide which
    // move to take. If we chose 1, for example, the tiger would only consider
    // cells next to it to decide where to move; this wouldn't probably be
    // what we want.
    // We want the tiger to think for infinite steps: this can be
    // approximated with a very high horizon, since in theory the final solution
    // will converge to a single policy anyway. Thus we put a very high number
    // as the horizon here.
    printCurrentTimeString(); std::cout << " - Solving MDP using infinite horizon...\n";
    AIToolbox::MDP::ValueIteration solver(1000000);

    // This is where the magic happen. This could take around 10-20 minutes,
    // depending on your machine (most of the time is spent on this tutorial's
    // code, however, since it is a pretty inefficient implementation).
    // But you can play with it and make it better!
    //
    // If you are using the Sparse Model though, it is instantaneous since
    // Eigen is very very efficient in computing the values we need!
    auto solution = solver(model);

    printCurrentTimeString();
    std::cout << " - Converged: " << (std::get<0>(solution) < solver.getTolerance()) << "\n";

    AIToolbox::MDP::Policy policy(world.getS(), world.getA(), std::get<1>(solution));

    // We create a random engine to pick a random state as start.
    std::default_random_engine rand(AIToolbox::Seeder::getSeed());
    std::uniform_int_distribution<size_t> start(0, SQUARE_SIZE * SQUARE_SIZE * SQUARE_SIZE * SQUARE_SIZE - 1);

    size_t s, a, s1;
    double r, totalReward = 0.0;

    // We create a starting state which is not the end.
    do s = start(rand);
    while (model.isTerminal(s));

    size_t t = 100;
    while (true) {
        // Print it!
        printState(decodeState(s));

        // We give the tiger a time limit, but if it reaches
        // the antelope we end the game.
        if (t == 0 || model.isTerminal(s)) break;

        // We sample an action for this state according to the optimal policy
        a = policy.sampleAction(s);
        // And we use the model to simulate what is going to happen next (in
        // case of a "real world" scenario where the library is used this step
        // would not exist as the world would automatically step to the next
        // state. Here we simulate.
        std::tie(s1, r) = model.sampleSR(s, a);

        // Add into the total reward (we don't use this here, it's just as an
        // example)
        totalReward += r;
        // Update the current state with the new one.
        s = s1;

        --t;
        goup(SQUARE_SIZE);

        // Sleep 1 second so the user can see what is happening.
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // You can save, and then load up this policy again on files. You will not
    // need to solve the model again ever, and you can embed the policy into
    // any application you want!
    // {
    //     std::ofstream output("policy.txt");
    //     output << policy;
    // }

    return 0;
}