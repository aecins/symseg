// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef VIS_HPP_
#define VIS_HPP_

#include <visualization/pcl_visualization.hpp>
#include <geometry/geometry.hpp>

// State
struct VisState
{
  VisState ()
    : cloudDisplay_ (CLOUD)
    , updateDisplay_ (true)
    , showNormals_ (false)
    , showOccupancy_ (false)
    , showSymmetry_ (true)
    , showReconstructedCloud_ (false)
    , segIterator_ (0)
    , instanceIdIt_ (0)    
    , pointSize_ (5.0)
    , showFullResolution_ (true)
  {};
  
  enum CloudDisplay   { CLOUD, INITIAL_OVERSEGMENTAION, ROTATIONAL_SYMMETRIES, ROTATIONAL_SEGMENTS,
                        CLOUD_AFTER_ROT, OVERSEGMENTATION_AFTER_ROT, REFLECTIONAL_SYMMETRIES, REFLECTIONAL_SEGMENTS, FINAL_SEGMENTS };
    
  CloudDisplay cloudDisplay_;
  bool updateDisplay_;
  bool showNormals_;
  bool showOccupancy_;
  bool showSymmetry_;
  bool showReconstructedCloud_;
  
  int segIterator_;
  int instanceIdIt_;
  
  float pointSize_;
  bool showFullResolution_;
};

// Callback
void keyboard_callback (const pcl::visualization::KeyboardEvent &event, void *cookie)
{
  VisState* visState = reinterpret_cast<VisState*> (cookie);
  
  if (event.keyUp ())
  {    
    std::string key = event.getKeySym ();
//     cout << key << " key pressed!\n";
    
    visState->updateDisplay_ = true;
    
    // Change segmentation stage displayed
    if (key == "KP_1")
      visState->cloudDisplay_ = VisState::CLOUD;
    else if (key == "KP_2")
      visState->cloudDisplay_ = VisState::INITIAL_OVERSEGMENTAION;
    else if (key == "KP_3")
      visState->cloudDisplay_ = VisState::ROTATIONAL_SYMMETRIES;
    else if (key == "KP_4")
      visState->cloudDisplay_ = VisState::ROTATIONAL_SEGMENTS;
    else if (key == "KP_5")
      visState->cloudDisplay_ = VisState::CLOUD_AFTER_ROT;
    else if (key == "KP_6")
      visState->cloudDisplay_ = VisState::OVERSEGMENTATION_AFTER_ROT;
    else if (key == "KP_7")
      visState->cloudDisplay_ = VisState::REFLECTIONAL_SYMMETRIES;
    else if (key == "KP_8")
      visState->cloudDisplay_ = VisState::REFLECTIONAL_SEGMENTS;
    else if (key == "KP_9")
      visState->cloudDisplay_ = VisState::FINAL_SEGMENTS;
        
    // Iterators
    else if (key == "Left")
      visState->segIterator_--;
    else if (key == "Right")
      visState->segIterator_++;
    
    else if (key == "Down")
      visState->instanceIdIt_--;
    else if (key == "Up")
      visState->instanceIdIt_++;

    // Point size
    else if (key == "KP_Add")
      visState->pointSize_ += 1.0;
    else if (key == "KP_Subtract")
      visState->pointSize_ = std::max(1.0, visState->pointSize_ - 1.0);    
        
    // Additional display
    else if (key == "KP_Decimal" || key == "KP_Delete")
      visState->showOccupancy_ = !visState->showOccupancy_;
    else if ((key == "n") || (key == "N"))
      visState->showNormals_ = !visState->showNormals_;
    else if ((key == "m") || (key == "M"))
      visState->showSymmetry_ = !visState->showSymmetry_;
    else if ((key == "Shift_L") || (key == "comma"))
      visState->showReconstructedCloud_ = !visState->showReconstructedCloud_;    
    
    // Resolution and color
    else if (key == "KP_Multiply")
      visState->showFullResolution_ = !visState->showFullResolution_;
    
    else
      visState->updateDisplay_ = false;
  }
}

#endif // VIS_HPP_