//
//  FeatureManager.cpp
//  MyTriangulation
//
//  Created by Yang Shuo on 26/3/15.
//  Copyright (c) 2015 Yang Shuo. All rights reserved.
//

#include "FeatureManager.h"
#include "g_param.h"
#include <iostream>
//
//FeatureManager::FeatureManager()
//{
//    current_frame = 0;
//}
//
//void FeatureManager::addFeatures(const vector<pair<int, Vector3d>> &image, SlideState _state)
//{
//    printf("input feature: %lu", image.size());
//    
//    // init is_lost
//    for (auto & item : feature_record_dict)
//    {
//        item.second.is_lost = false;
//    }
//    
//    // add sliding state
//    // removeSlideState if the window is full already
//    if (current_frame == SLIDING_WINDOW_SIZE-1)
//    {
//        removeSlideState(1);
//    }
//    // else insert directly
//    else
//    {
//        slidingWindow.push_back(_state);
//        current_frame++;
//    }
//    
//    
//    for (auto & id_pts : image)
//    {
//        int   id = id_pts.first;
//        float x = id_pts.second(0);
//        float y = id_pts.second(1);
//        float z = id_pts.second(2);
//        
//        if (feature_record_dict.find(id) == feature_record_dict.end())
//        {
//            
//        }
//        else
//        {
//            
//        }
//    }
//}
//
//void FeatureManager::addSlideState(SlideState _state)
//{
//
//}
//
//void FeatureManager::removeSlideState(int index)
//{
//    /* 1. remove sliding window */
//    std::list<SlideState>::iterator itr;
//    // make sure index in range 0 to SLIDING_WINDOW_SIZE-1
//    if (index < 0 || index > SLIDING_WINDOW_SIZE-1)
//    {
//        // invalid input
//        return;
//    }
//    
//    if (index > current_frame)
//    {
//        //TODO: throw an error
//        return;
//    }
//    
//    itr = slidingWindow.begin();
//    for (int idx = 0; idx<index;idx++)
//    {
//        itr++;
//    }
//    
//    slidingWindow.erase(itr);
//    current_frame--;
//    
//    /* 2. remove feature record */
//    for (auto & item : feature_record_dict)
//    {
//        if (item.second.start_frame<index)
//        {
//            //item.second.feature_points.erase()
//        }
//    }
//    
//    
//}
//
//void FeatureManager::debugOut()
//{
//    std::cout<< "sliding window" << std::endl;
//    for (std::list<SlideState>::iterator it = slidingWindow.begin();
//         it!= slidingWindow.end();++it)
//    {
//        std::cout<< it->q << std::endl
//                 << it->p << std::endl
//                 << it->v << std::endl;
//    }
//}