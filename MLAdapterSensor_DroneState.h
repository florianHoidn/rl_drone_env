#pragma once

#include "CoreMinimal.h"
#include "Sensors/MLAdapterSensor.h"
#include "MLAdapterTypes.h"
#include "ContinuousControlPawn.h"
#include "MultirotorPhysics.hpp"
#include "MLAdapterSensor_DroneState.generated.h"


UCLASS()
class RL_DRONE_ENV_API UMLAdapterSensor_DroneState : public UMLAdapterSensor
{
	GENERATED_BODY()

public:
	UMLAdapterSensor_DroneState(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

protected:
	virtual TSharedPtr<FMLAdapter::FSpace> ConstructSpaceDef() const override;
	virtual void UpdateSpaceDef() override;
	virtual void OnAvatarSet(AActor* Avatar) override;
	virtual void SenseImpl(const float DeltaTime) override;
	virtual void GetObservations(FMLAdapterMemoryWriter& Ar) override;

	AContinuousControlPawn* pawn = nullptr;
	double drone_state_features[DroneState::DIM];
	
};
