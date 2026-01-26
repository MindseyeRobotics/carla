// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/ShaderBasedSensor.h"
#include "Carla/Actor/ActorDefinition.h"
#include "IRCamera.generated.h"

/// Sensor that produces thermal/infrared images.
/// Based on FLIR Lepton 3.5 specifications:
/// - Resolution: 160x120 pixels
/// - Spectral Range: 8-14 µm (long-wave infrared)
/// - Frame Rate: Up to 9 Hz
/// - Output: 14-bit thermal data (converted to 8-bit for visualization)
UCLASS()
class CARLA_API AIRCamera : public AShaderBasedSensor
{
  GENERATED_BODY()

public:

  static FActorDefinition GetSensorDefinition();

  AIRCamera(const FObjectInitializer &ObjectInitializer);

protected:

  void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds) override;
};
