#include "MLAdapterSensor_DroneState.h"

UMLAdapterSensor_DroneState::UMLAdapterSensor_DroneState(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer) {
	TickPolicy = EMLAdapterTickPolicy::EveryTick;
}

void UMLAdapterSensor_DroneState::SenseImpl(const float DeltaTime) {
	if (this->pawn) {
		const DroneState& drone_state = this->pawn->getDroneState();

		this->drone_state_features[0] = drone_state.position.x;
		this->drone_state_features[1] = drone_state.position.y;
		this->drone_state_features[2] = drone_state.position.z;

		this->drone_state_features[3] = drone_state.orientation.x;
		this->drone_state_features[4] = drone_state.orientation.y;
		this->drone_state_features[5] = drone_state.orientation.z;
		this->drone_state_features[6] = drone_state.orientation.w;

		this->drone_state_features[7] = drone_state.linear_velocity.x;
		this->drone_state_features[8] = drone_state.linear_velocity.y;
		this->drone_state_features[9] = drone_state.linear_velocity.z;

		this->drone_state_features[10] = drone_state.angular_velocity.x;
		this->drone_state_features[11] = drone_state.angular_velocity.y;
		this->drone_state_features[12] = drone_state.angular_velocity.z;
	}
}

void UMLAdapterSensor_DroneState::OnAvatarSet(AActor* Avatar) {
	Super::OnAvatarSet(Avatar);
	this->pawn = Cast<AContinuousControlPawn>(Avatar);
	if (this->pawn) {
		this->SenseImpl(0.f);
		this->UpdateSpaceDef();
	}
}

void UMLAdapterSensor_DroneState::GetObservations(FMLAdapterMemoryWriter& Ar) {
	FScopeLock Lock(&ObservationCS);
	FMLAdapter::FSpaceSerializeGuard SerializeGuard(SpaceDef, Ar);
	// TODO sending double over the wire isn't really supported, unfortunately.
	Ar.Serialize(this->drone_state_features, DroneState::DIM * sizeof(double));
}

TSharedPtr<FMLAdapter::FSpace> UMLAdapterSensor_DroneState::ConstructSpaceDef() const {
	if (this->pawn) {
		return MakeShareable(new FMLAdapter::FSpace_Box({ this->pawn->observation_space_dim }));
	}
	return MakeShareable(new FMLAdapter::FSpace_Box({ 0 }));
}

void UMLAdapterSensor_DroneState::UpdateSpaceDef() {
	Super::UpdateSpaceDef();
}

