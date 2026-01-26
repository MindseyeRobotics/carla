// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/IRCamera.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

FActorDefinition AIRCamera::GetSensorDefinition()
{
  auto Definition = UActorBlueprintFunctionLibrary::MakeCameraDefinition(TEXT("ir"));
  
  // FLIR Lepton 3.5 specific attributes
  FActorVariation Resolution;
  Resolution.Id = TEXT("image_size_x");
  Resolution.Type = EActorAttributeType::Int;
  Resolution.RecommendedValues = { TEXT("160") };
  Resolution.bRestrictToRecommended = false;
  Definition.Variations.Add(Resolution);
  
  Resolution.Id = TEXT("image_size_y");
  Resolution.RecommendedValues = { TEXT("120") };
  Definition.Variations.Add(Resolution);
  
  // Frame rate typical for FLIR Lepton 3.5
  FActorVariation FrameRate;
  FrameRate.Id = TEXT("sensor_tick");
  FrameRate.Type = EActorAttributeType::Float;
  FrameRate.RecommendedValues = { TEXT("0.0") }; // Default: as fast as possible
  FrameRate.bRestrictToRecommended = false;
  Definition.Variations.Add(FrameRate);
  
  return Definition;
}

AIRCamera::AIRCamera(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  // Apply lens distortion similar to real thermal camera
  AddPostProcessingMaterial(
      TEXT("Material'/Carla/PostProcessingMaterials/PhysicLensDistortion.PhysicLensDistortion'"));
  
  // Apply thermal/IR effect material
  // Note: This uses depth as a proxy for thermal emission
  // In reality, thermal cameras detect infrared radiation based on object temperature
  AddPostProcessingMaterial(
#if PLATFORM_LINUX
      TEXT("Material'/Carla/PostProcessingMaterials/DepthEffectMaterial_GLSL.DepthEffectMaterial_GLSL'")
#else
      TEXT("Material'/Carla/PostProcessingMaterials/DepthEffectMaterial.DepthEffectMaterial'")
#endif
  );
}

void AIRCamera::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AIRCamera::PostPhysTick);
  Super::PostPhysTick(World, TickType, DeltaSeconds);

  if (!AreClientsListening())
      return;

  auto FrameIndex = FCarlaEngine::GetFrameCounter();
  ImageUtil::ReadSensorImageDataAsyncFColor(*this, [this, FrameIndex](
    TArrayView<const FColor> Pixels,
    FIntPoint Size) -> bool
  {
    SendDataToClient(*this, Pixels, FrameIndex);
    return true;
  });
}
