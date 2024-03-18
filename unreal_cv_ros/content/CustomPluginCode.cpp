/** Code for the custom uecvros_full command for the unrealcv plugin. Copy the following 3 code snippets into their described locations.

/** 1. Add this declaration to CameraHandler.h */

/** Run a step of the uecvros routine */
FExecStatus UecvrosFull(const TArray<FString>& Args);


/** 2. Add the command dispatcher in CameraHandler.cpp */

CommandDispatcher->BindCommand(
	"vget /uecvros/full [float] [float] [float] [float] [float] [float] [float] [uint]", FDispatcherDelegate::CreateRaw(this, &FCameraHandler::UecvrosFull),
	"Run the full UECVROS routine [X, Y, Z, p, y, r, coll, cameraID] Return npy image unless collided"
);

/** 3. Add the function body to CameraHandler.cpp */

FExecStatus FCameraHandler::UecvrosFull(const TArray<FString>& Args)
{
	if (Args.Num() == 8) // [X, Y, Z, p, y, r, collisionT, cameraID]
	{
		float X = FCString::Atof(*Args[0]), Y = FCString::Atof(*Args[1]), Z = FCString::Atof(*Args[2]);
		FVector LocNew = FVector(X, Y, Z);
		
		float p = FCString::Atof(*Args[3]), y = FCString::Atof(*Args[4]), r = FCString::Atof(*Args[5]);
		FRotator RotNew = FRotator(p, y, r);
		
		float tol = FCString::Atof(*Args[6]);
		bool sweep = tol > 0;	// Is collision enabled?
		
		// produce images and stack binary data
		TArray<uint8> Data;
		TArray<FString> CamIdArgs;
		CamIdArgs.Init(Args[7], 1);
		CamIdArgs.Add(TEXT("png"));
		Data += (this->GetCameraLit(CamIdArgs)).GetData();
		CamIdArgs.Remove(TEXT("png"));
		CamIdArgs.Add(TEXT("npy"));
		Data += (this->GetCameraDepth(CamIdArgs)).GetData();
		
		// Set new position & orientation
		APawn* Pawn = FUnrealcvServer::Get().GetPawn();
		FHitResult* hitresult = new FHitResult();
		Pawn->SetActorLocation(LocNew, sweep, hitresult, ETeleportType::TeleportPhysics);
		Pawn->SetActorRotation(RotNew, ETeleportType::TeleportPhysics);
		AController* Controller = Pawn->GetController();
		Controller->ClientSetRotation(RotNew, true);	// the true is important for performance/blurr

		// Return results
		if (sweep && hitresult->IsValidBlockingHit())
		{
			return FExecStatus::OK(FString::Printf(TEXT("Collision detected!")));		// returns OK to get the message string only}
		}
		else 
		{
			return FExecStatus::Binary(Data);
		}
	}
	return FExecStatus::InvalidArgument;
}

