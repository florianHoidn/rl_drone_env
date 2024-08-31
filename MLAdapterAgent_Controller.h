#pragma once

#include "CoreMinimal.h"
#include "Agents/MLAdapterAgent.h"
#include "MLAdapterSpace.h"
#include <vector>
#include "MLAdapterAgent_Controller.generated.h"

UCLASS()
class RL_DRONE_ENV_API UMLAdapterAgent_Controller : public UMLAdapterAgent
{
	GENERATED_BODY()

public:

	//UMLAdapterAgent_Controller(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());
	
	virtual void Act(const float DeltaTime) override;

	virtual void DigestActions(FMLAdapterMemoryReader& ValueStream) override;

	virtual void GetActionSpaceDescription(FMLAdapterSpaceDescription& OutSpaceDesc) const override;

	virtual float GetReward() const override;
	virtual bool IsDone() const override;

protected:
	void init() const; // Makes almost no sense to have this const, but I need to call it in GetActionSpaceDescription.
	mutable int action_space_dim = 0;
	mutable FCriticalSection inputLock;
	mutable std::vector<double> action_input;
	mutable TArray<float> rawData;

};
