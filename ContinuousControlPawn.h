#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include <vector>
#include "MultirotorPhysics.hpp"

#include "ContinuousControlPawn.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogUnrealEditorDroneController, Log, All);

UCLASS()
class RL_DRONE_ENV_API AContinuousControlPawn : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AContinuousControlPawn();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	inline bool needsReset() const;

	inline float computeReward() const;
	
	inline const DroneState& getDroneState() const {
		return this->multirotor_physics.get_current_drone_state();
	}

	int action_space_dim = -1;
	std::vector<double> action_input;
	uint32 observation_space_dim = 0;

protected:

	void init(UStaticMeshComponent* skeletal_mesh);

	virtual void Reset() override;

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called when the game ends.
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	std::atomic<bool> is_initialized = false;

private:
	MultirotorPhysics multirotor_physics;
	UStaticMeshComponent* mesh_component = nullptr;
	linalg::vec3 orig_root_pos;
	linalg::quat orig_root_rot;
	
	std::atomic<bool> needs_reset = false;
};
