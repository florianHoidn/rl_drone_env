#include "MLAdapterAgent_Controller.h"
#include "ContinuousControlPawn.h"


void UMLAdapterAgent_Controller::init() const {
	AContinuousControlPawn* pawn = Cast<AContinuousControlPawn>(GetAvatar());
	if (pawn == nullptr) {
		// TODO this shouldn't happen when the game actually starts (only potentially when the editor opens or so)
		return;
	}
	this->action_space_dim = pawn->action_space_dim;
	for (size_t i = 0; i < this->action_space_dim; i++) {
		this->action_input.push_back(0.0);
	}
	this->rawData.SetNum(this->action_space_dim);
}

void UMLAdapterAgent_Controller::GetActionSpaceDescription(FMLAdapterSpaceDescription& OutSpaceDesc) const {
	if (this->action_space_dim <= 0) {
		// TODO not cool. Maybe do this somewhere else to get rid of the const.
		this->init();
	}
	FMLAdapterDescription ElementDesc;
	//FMLAdapter::FSpace* Result = new FMLAdapter::FSpace_Discrete(RegisteredKeys.Num());
	//MakeShareable(FMLAdapter::FSpace_Box(RegisteredKeys.Num()))
	ElementDesc.Add(FMLAdapter::FSpace_Box({ (uint32)(this->action_space_dim) }, -1.f, 1.f));
	OutSpaceDesc.Add(TEXT("continuous_input"), ElementDesc);
}

void UMLAdapterAgent_Controller::Act(const float DeltaTime) {
	AContinuousControlPawn* pawn = Cast<AContinuousControlPawn>(GetAvatar());
	if (pawn == nullptr) {
		return;
	}
	FScopeLock Lock(&inputLock);
	pawn->action_input = action_input;
}

void UMLAdapterAgent_Controller::DigestActions(FMLAdapterMemoryReader& ValueStream) {
	ValueStream.Serialize(rawData.GetData(), rawData.Num() * sizeof(float));
	{
		FScopeLock Lock(&inputLock);
		action_input.clear();
		for (size_t i = 0; i < action_space_dim; i++) {
			action_input.push_back(rawData[i]);
		}
	}
}

float UMLAdapterAgent_Controller::GetReward() const {
	AContinuousControlPawn* pawn = Cast<AContinuousControlPawn>(GetAvatar());
	if (pawn) {
		return pawn->computeReward();
	}
	return 0.f;
}

bool UMLAdapterAgent_Controller::IsDone() const {
	AContinuousControlPawn* pawn = Cast<AContinuousControlPawn>(GetAvatar());
	if (pawn) {
		return pawn->needsReset();
	}
	return true;
}


