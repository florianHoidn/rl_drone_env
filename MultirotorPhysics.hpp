#pragma once

#include "CoreMinimal.h"
#include "linalg.hpp"

DECLARE_LOG_CATEGORY_EXTERN(LogUnrealEditorDronePhysics, Log, All);

struct DroneState {
	static constexpr int DIM = 13; // Sum of array sizes below, 3 + 4 + 3 + 3
	linalg::vec3 position = { 0.0, 0.0, 0.0 };
    linalg::quat orientation = { 0.0, 0.0, 0.0, 1.0 };
    linalg::vec3 linear_velocity = { 0.0, 0.0, 0.0 };
    linalg::vec3 angular_velocity = { 0.0, 0.0, 0.0 };
};

struct DroneControlAction {
    static constexpr int DIM = 4;
    static constexpr double MIN_RPM = 0.0;
    static constexpr double MAX_RPM = 21702.0;
    static constexpr double ONE_OVER_RANGE = 1.0 / (MAX_RPM - MIN_RPM);
    static constexpr double STABLE_HOVER_BIAS = 0.334 * (MAX_RPM - MIN_RPM);


	double rmps_per_rotor[DroneControlAction::DIM];
};

struct DroneSpec {
	// TODO this stuff should be more flexible
	const size_t num_rotors = 4;

	const linalg::vec3 rpm_to_thrust_coefs = { 0.0, 0.0, 3.16e-10 };
	const double rpm_to_torque_coef = 0.005964552;
	const double drone_mass = 0.027;

	const linalg::vec3 rotor_thrust_directions[4] = { { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 } };
	const linalg::vec3 rotor_torque_directions[4] = { { 0.0, 0.0, -1.0 }, { 0.0, 0.0, +1.0 }, { 0.0, 0.0, -1.0 }, { 0.0, 0.0, +1.0 } };
	const linalg::vec3 rotor_positions[4] = { { 0.028, -0.028, 0 }, { -0.028, -0.028, 0 }, { -0.028, 0.028, 0 }, { 0.028, 0.028, 0 } };

    const linalg::mat3x3 J = {
        3.85e-6,
        0.0,
        0.0,

        0.0,
        3.85e-6,
        0.0,

        0.0,
        0.0,
        5.9675e-6
    };
    const linalg::mat3x3 J_inv = {
        259740.2597402597,
        0.0,
        0.0,

        0.0,
        259740.2597402597,
        0.0,

        0.0,
        0.0,
        167574.36112274823
    };
};

/**
 * Simulates the physics of a multicopter drone accurately enough 
 * to allow for RL training of autonomous drone controllers within the 
 * Unreal engine.
 */
class RL_DRONE_ENV_API MultirotorPhysics {
public:
	MultirotorPhysics();
	~MultirotorPhysics();

    void init(const DroneState& initial_state) {
        this->current_state = initial_state;
    }

	/**
	* Apply the drone control action to the current state, advance the multirotor physics by dt, 
	* and return (an rk4 approximation of) the resulting state.
	*/
	// TODO maybe I should use a fixed simulation update rate and make sure that the simulation runs for as many steps as needed to match ue5's tick rate.
	void apply_control(const DroneControlAction& action, const double dt);

    const linalg::vec3& get_position() const {
        return this->current_state.position;
    }

    const linalg::quat& get_orientation() const {
        return this->current_state.orientation;
    }

    const DroneState& get_current_drone_state() const {
        return this->current_state;
    }

    const DroneState& get_prev_drone_state() const {
        return this->prev_state;
    }

    double get_prev_to_curr_dt() const {
        return this->prev_to_curr_dt;
    }

    DroneControlAction get_prev_action() const {
        return this->prev_action;
    }

private:

	DroneState physics_step(const DroneState& current_state, const DroneControlAction& action) const;

    static inline void state_add_accumulate(const DroneState& s, DroneState& out) {
        linalg::add_accumulate(s.position, out.position);
        linalg::add_accumulate(s.orientation, out.orientation);
        linalg::add_accumulate(s.linear_velocity, out.linear_velocity);
        linalg::add_accumulate(s.angular_velocity, out.angular_velocity);
    };

    static inline void state_scalar_multiply(const DroneState& s, const double scalar, DroneState& out) {
        linalg::scalar_multiply(s.position, scalar, out.position);
        linalg::scalar_multiply(s.orientation, scalar, out.orientation);
        linalg::scalar_multiply(s.linear_velocity, scalar, out.linear_velocity);
        linalg::scalar_multiply(s.angular_velocity, scalar, out.angular_velocity);
    };

    static inline void state_scalar_multiply(DroneState& s, const double scalar) {
        linalg::scalar_multiply(s.position, scalar);
        linalg::scalar_multiply(s.orientation, scalar);
        linalg::scalar_multiply(s.linear_velocity, scalar);
        linalg::scalar_multiply(s.angular_velocity, scalar);
    };

    static inline void state_scalar_multiply_accumulate(const DroneState& s, const double scalar, DroneState& out) {
        linalg::scalar_multiply_accumulate(s.position, scalar, out.position);
        linalg::scalar_multiply_accumulate(s.orientation, scalar, out.orientation);
        linalg::scalar_multiply_accumulate(s.linear_velocity, scalar, out.linear_velocity);
        linalg::scalar_multiply_accumulate(s.angular_velocity, scalar, out.angular_velocity);
    };

    static inline void normalize_state(DroneState& state) {
        const double quaternion_norm = sqrt(state.orientation.x * state.orientation.x 
                                + state.orientation.y * state.orientation.y 
                                + state.orientation.z * state.orientation.z 
                                + state.orientation.w * state.orientation.w);
        state.orientation.x /= quaternion_norm;
        state.orientation.y /= quaternion_norm;
        state.orientation.z /= quaternion_norm;
        state.orientation.w /= quaternion_norm;
    }

    DroneControlAction prev_action;
    DroneState prev_state;
    double prev_to_curr_dt = 0.0;
    DroneState current_state;


	const DroneSpec drone_spec;
    const linalg::vec3 gravity = { 0.0, 0.0, -9.81 };
};