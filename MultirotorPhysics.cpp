#include "MultirotorPhysics.hpp"

DEFINE_LOG_CATEGORY(LogUnrealEditorDronePhysics);

MultirotorPhysics::MultirotorPhysics() {
}

MultirotorPhysics::~MultirotorPhysics() {
}

void MultirotorPhysics::apply_control(const DroneControlAction& action, const double dt) {

    this->prev_action = action;
    this->prev_state = this->current_state;
    this->prev_to_curr_dt = dt;

    DroneState k1 = this->physics_step(this->current_state, action);
    DroneState var;
    state_scalar_multiply(k1, dt * 0.5, var);
    {
        state_add_accumulate(current_state, var);
        DroneState k2 = this->physics_step(var, action);
        state_scalar_multiply(k2, dt * 0.5, var);
        state_scalar_multiply_accumulate(k2, 2, k1);
    }
    {
        state_add_accumulate(current_state, var);
        DroneState k3 = this->physics_step(var, action);
        state_scalar_multiply(k3, dt, var);
        state_scalar_multiply_accumulate(k3, 2, k1);
    }
    {
        state_add_accumulate(current_state, var);
        DroneState k4 = this->physics_step(var, action);
        state_add_accumulate(k4, k1);
    }
    state_scalar_multiply(k1, dt / 6.0);
    state_add_accumulate(current_state, k1);

    normalize_state(k1);

    this->current_state = k1;
}

DroneState MultirotorPhysics::physics_step(const DroneState& state, const DroneControlAction& action) const {

    DroneState next_state = state; // TODO not sure if I need to copy here.

	// See rl_tools::rl::environments::multirotor::multirotor_dynamics
    linalg::vec3 thrust = { .0f, .0f, .0f };
    linalg::vec3 torque = { .0f, .0f, .0f };

    for (size_t rotor_idx = 0; rotor_idx < this->drone_spec.num_rotors; rotor_idx++) {

        double rpm = action.rmps_per_rotor[rotor_idx];
        double thrust_magnitude = this->drone_spec.rpm_to_thrust_coefs.x + this->drone_spec.rpm_to_thrust_coefs.y * rpm + this->drone_spec.rpm_to_thrust_coefs.z * rpm * rpm;
        linalg::vec3 thrust_vec;
        linalg::scalar_multiply(this->drone_spec.rotor_thrust_directions[rotor_idx], thrust_magnitude, thrust_vec);
        linalg::add_accumulate(thrust_vec, thrust);

        linalg::scalar_multiply_accumulate(this->drone_spec.rotor_torque_directions[rotor_idx], thrust_magnitude * this->drone_spec.rpm_to_torque_coef, torque);
        linalg::cross_product_accumulate(this->drone_spec.rotor_positions[rotor_idx], thrust_vec, torque);
    }

    next_state.position.x = current_state.linear_velocity.x;
    next_state.position.y = current_state.linear_velocity.y;
    next_state.position.z = current_state.linear_velocity.z;

    linalg::quaternion_derivative(current_state.orientation, current_state.angular_velocity, next_state.orientation);
    linalg::rotate_vector_by_quaternion(current_state.orientation, thrust, next_state.linear_velocity);
    
    linalg::scalar_multiply(next_state.linear_velocity, 1.0 / this->drone_spec.drone_mass);
    linalg::add_accumulate(this->gravity, next_state.linear_velocity);

    linalg::vec3 vector = { 0.0, 0.0, 0.0 };
    linalg::vec3 vector2 = { 0.0, 0.0, 0.0 };
    linalg::matrix_vector_product(this->drone_spec.J, current_state.angular_velocity, vector);
    linalg::cross_product(current_state.angular_velocity, vector, vector2);
    linalg::sub(torque, vector2, vector);
    linalg::matrix_vector_product(this->drone_spec.J_inv, vector, next_state.angular_velocity);

    return next_state;
}
