#include "ContinuousControlPawn.h"
#include <cmath>

DEFINE_LOG_CATEGORY(LogUnrealEditorDroneController);

// Sets default values
AContinuousControlPawn::AContinuousControlPawn() {
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root Component"));
}

void AContinuousControlPawn::init(UStaticMeshComponent* mesh) {

	this->mesh_component = mesh;

	this->action_space_dim = 4;
	this->observation_space_dim = DroneState::DIM;

	const FVector& root_pos = this->mesh_component->GetComponentLocation();
	const FQuat& root_rot = this->mesh_component->GetComponentQuat();

	this->orig_root_pos = { root_pos.X, root_pos.Y, root_pos.Z };
	this->orig_root_rot = { root_rot.X, root_rot.Y, root_rot.Z, root_rot.W };
	this->multirotor_physics.init({
			this->orig_root_pos, /* position */
			this->orig_root_rot, /* orientation */
			{ 0.0, 0.0, 0.0 }, /* linear_velocity */
			{ 0.0, 0.0, 0.0 } /* angular_velocity */
		});

	this->is_initialized = true;
}

// Called when the game starts or when spawned
void AContinuousControlPawn::BeginPlay() {
	Super::BeginPlay();
	
}

void AContinuousControlPawn::EndPlay(const EEndPlayReason::Type EndPlayReason) {
	Super::EndPlay(EndPlayReason);
	this->is_initialized = false;
}

// Called every frame
void AContinuousControlPawn::Tick(float DeltaTime) {

	if (!this->is_initialized) {
		UStaticMeshComponent* skeletal_mesh = this->FindComponentByClass<UStaticMeshComponent>();
		if (skeletal_mesh) {
			this->init(skeletal_mesh);
		}
	}
	if (this->action_input.empty()) {
		return;
	}

	// TODO avoid action copy
	this->multirotor_physics.apply_control({ { this->action_input[0], this->action_input[1], this->action_input[2], this->action_input[3] } }, DeltaTime);

	const linalg::vec3& new_pos = this->multirotor_physics.get_position();
	const linalg::quat& new_rot = this->multirotor_physics.get_orientation();

	const FVector ue_pos = { new_pos.x, new_pos.y, new_pos.z };
	const FQuat ue_rot = { new_rot.x, new_rot.y, new_rot.z, new_rot.w };

	UE_LOG(LogUnrealEditorDroneController, Display, TEXT("Tick update: position=%s, rotation=%s"), *(ue_pos.ToString()), *(ue_rot.ToString()));

	FHitResult collisions;
	this->mesh_component->SetWorldLocationAndRotation(ue_pos, ue_rot, false, &collisions, ETeleportType::TeleportPhysics);
	if (collisions.bBlockingHit) {
		this->needs_reset = true;
	}

	Super::Tick(DeltaTime);
}

// Called to bind functionality to input
void AContinuousControlPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) {
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

inline bool AContinuousControlPawn::needsReset() const {

	return needs_reset;
}

inline float AContinuousControlPawn::computeReward() const {

	if (!is_initialized) {
		return 0.f;
	}

	//if (this->needs_reset) {
	//	return 0.0;
	//}
	double rew = 0.0;

	//const DroneState& prev_state = this->multirotor_physics.get_prev_drone_state();
	const DroneState& current_state = this->multirotor_physics.get_current_drone_state();

	rew -= 5.0 * (1 - current_state.orientation.w * current_state.orientation.w);
	
	rew -= 5.0 * (std::abs(this->orig_root_pos.x - current_state.position.x)
			+ std::abs(this->orig_root_pos.y - current_state.position.y)
			+ std::abs(this->orig_root_pos.z - current_state.position.z));

	rew -= 0.01 * (std::abs(current_state.linear_velocity.x)
					+ std::abs(current_state.linear_velocity.y)
					+ std::abs(current_state.linear_velocity.z));

	//rew -= 0.0 * std::abs(current_state.angular_velocity.x)
	//							 + std::abs(current_state.angular_velocity.y)
	//							 + std::abs(current_state.angular_velocity.z);

	//linalg::vec3 linear_acc;
	//linalg::sub(current_state.linear_velocity, prev_state.linear_velocity, linear_acc);
	//const double dt_sq = this->multirotor_physics.get_prev_to_curr_dt() * this->multirotor_physics.get_prev_to_curr_dt();
	//rew -= (std::abs(linear_acc.x) + std::abs(linear_acc.y) + std::abs(linear_acc.z)) / dt_sq;
	
	//linalg::vec3 angular_acc;
	//linalg::sub(current_state.angular_velocity, prev_state.angular_velocity, angular_acc);
	//rew -= (std::abs(angular_acc.x) + std::abs(angular_acc.y) + std::abs(angular_acc.z)) / dt_sq;

	double control_bias_cost = 0.0;
	const DroneControlAction& prev_action = this->multirotor_physics.get_prev_action();
	for (int i = 0; i < this->action_space_dim; i++) {
		double diff = (prev_action.rmps_per_rotor[i] - DroneControlAction::STABLE_HOVER_BIAS) * DroneControlAction::ONE_OVER_RANGE;
		control_bias_cost += diff * diff;
	}

	rew -= 0.01 * control_bias_cost;
	return 0.5 * rew + 2.0;
}

void AContinuousControlPawn::Reset() {
	Super::Reset();
	if (this->is_initialized) {
		this->multirotor_physics.init({
			this->orig_root_pos, /* position */
			this->orig_root_rot, /* orientation */
			{ 0.0, 0.0, 0.0 }, /* linear_velocity */
			{ 0.0, 0.0, 0.0 } /* angular_velocity */
		});

		const linalg::vec3& new_pos = this->multirotor_physics.get_position();
		const linalg::quat& new_rot = this->multirotor_physics.get_orientation();

		const FVector ue_pos = { new_pos.x, new_pos.y, new_pos.z };
		const FQuat ue_rot = { new_rot.x, new_rot.y, new_rot.z, new_rot.w };

		this->mesh_component->SetWorldLocationAndRotation(ue_pos, ue_rot, false, nullptr, ETeleportType::TeleportPhysics);
	}
}

