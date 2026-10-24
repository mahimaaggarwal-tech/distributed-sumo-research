/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2023 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSEdgeControl.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 09 Apr 2001
///
// Stores edges and lanes, performs moving of vehicle
/****************************************************************************/
#include <config.h>

#include <iostream>
#include <queue>
#include <vector>
#include "MSEdgeControl.h"
#include "MSVehicleControl.h"
#include "MSGlobals.h"
#include "MSEdge.h"
#include "MSLane.h"
#include "MSVehicle.h"

#define PARALLEL_PLAN_MOVE
#define PARALLEL_EXEC_MOVE
//#define PARALLEL_CHANGE_LANES
//#define LOAD_BALANCING

//#define PARALLEL_STOPWATCH
//Mahima
#define QSUMO_GLOBAL
// ===========================================================================
// member method definitions
// ===========================================================================
MSEdgeControl::MSEdgeControl(const std::vector< MSEdge* >& edges)
    : myEdges(edges),
      myLanes(MSLane::dictSize()),
      myWithVehicles2Integrate(MSGlobals::gNumSimThreads > 1),
      myLastLaneChange(edges.size()),
      myInactiveCheckCollisions(MSGlobals::gNumSimThreads > 1),
      myMinLengthGeometryFactor(1.),
#ifdef THREAD_POOL
      myThreadPool(false, std::vector<int>(MSGlobals::gNumThreads, 0)),
#endif
      myStopWatch(3) {
    // build the usage definitions for lanes
    for (MSEdge* const edge : myEdges) {
        const std::vector<MSLane*>& lanes = edge->getLanes();
        if (!edge->hasLaneChanger()) {
            const int pos = lanes.front()->getNumericalID();
            myLanes[pos].lane = lanes.front();
            myLanes[pos].amActive = false;
            myLanes[pos].haveNeighbors = false;
            myMinLengthGeometryFactor = MIN2(edge->getLengthGeometryFactor(), myMinLengthGeometryFactor);
        } else {
            for (MSLane* const l : lanes) {
                const int pos = l->getNumericalID();
                myLanes[pos].lane = l;
                myLanes[pos].amActive = false;
                myLanes[pos].haveNeighbors = true;
                myMinLengthGeometryFactor = MIN2(l->getLengthGeometryFactor(), myMinLengthGeometryFactor);
            }
            myLastLaneChange[edge->getNumericalID()] = -1;
        }
    }
#ifndef THREAD_POOL
#ifdef HAVE_FOX
    if (MSGlobals::gNumThreads > 1) {
        while (myThreadPool.size() < MSGlobals::gNumThreads) {
            new WorkerThread(myThreadPool);
        }
    }
#endif
#endif
}


MSEdgeControl::~MSEdgeControl() {
#ifndef THREAD_POOL
#ifdef HAVE_FOX
    myThreadPool.clear();
#endif
#endif
#ifdef PARALLEL_STOPWATCH
    StopWatch<std::chrono::nanoseconds> wPlan;
    for (MSEdge* const edge : myEdges) {
        for (MSLane* const l : edge->getLanes()) {
            wPlan.add(l->getStopWatch()[0]);
        }
    }
    std::cout << wPlan.getHistory().size() << " lane planmove calls, average " << wPlan.getAverage() << " ns, total " << wPlan.getTotal() / double(1e9) << " s" << std::endl;
    std::cout << myStopWatch[0].getHistory().size() << " planmove calls, average " << myStopWatch[0].getAverage() << " ns, total " << myStopWatch[0].getTotal() / double(1e9) << " s" << std::endl;
    std::cout << myStopWatch[1].getHistory().size() << " execmove calls, average " << myStopWatch[1].getAverage() << " ns, total " << myStopWatch[1].getTotal() / double(1e9) << " s" << std::endl;
#endif
}

void
MSEdgeControl::setActiveLanes(std::list<MSLane*> lanes) {
    myActiveLanes = lanes;
    for (MSLane* lane : lanes) {
        myLanes[lane->getNumericalID()].amActive = true;
    }
}

//Mahima
void
MSEdgeControl::ResetGlobalBruttoLanes() {
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
        (*i)->ResetGlobalvector();
        ++i;
    }
    //myActiveLanes.clear();
}

void
MSEdgeControl::initBruttoAllEdges(lane_info* lane_arr) {
    for (MSEdge* e : myEdges) {
        e->inferEdgeType();
        const std::vector<MSLane*>& lanes = e->getLanes();
        for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
            //(*j)->updateGlobalBrutto(0, 0);
            if(!std::get<4>(lane_arr[(*i)->getNumericalID()]))
                    std::get<3>(lane_arr[(*i)->getNumericalID()]) = (*i)->getBruttoVehLenSum() + std::get<0>(lane_arr[(*i)->getNumericalID()]);
        
            //std::get<3>(lane_arr[(*j)->getNumericalID()]) = (*j)->getBruttoVehLenSum(); 
        }
    }
}

#ifdef close
//Mahima
void
MSEdgeControl::patchActiveLanesGlobal(std::map<int, std::vector<std::tuple<std::string, double, double, double, double, double, bool>>> &VehParam) {
    for (auto v : VehParam)
    {
        //std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  v.first << "\n";
        std::vector<std::tuple<std::string, double, double, double, double, double, bool>> vehp = v.second;
        int lane_id = v.first;
        LaneUsage& lu = myLanes[v.first];
        MSLane* i = lu.lane;  
        for(auto jj=0; jj<vehp.size(); ++jj)
        {
            std::string veh = std::get<0>(vehp[jj]).c_str();
            double pos = std::get<2>(vehp[jj]);
            double speed = std::get<1>(vehp[jj]);
            double acceleration = std::get<3>(vehp[jj]);
            double latoff = std::get<4>(vehp[jj]);
            double bruttoVehlenSum = std::get<5>(vehp[jj]);
            bool partial_veh = std::get<6>(vehp[jj]);
            std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  (i)->getID() << "veh:" << veh << "pos" << pos  << "partial_veh:" << partial_veh << "\n";
            i->updateGlobalvehicles(veh, pos, speed, acceleration, latoff, bruttoVehlenSum, partial_veh);
        }
            
        // if the lane was inactive but is now...
        if (!lu.amActive && vehp.size() > 0) {
            // ... add to active lanes and mark as such
            if (lu.haveNeighbors) {
                myActiveLanes.push_front(i);
                //std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  (i)->getID() << "\n";
            } else {
                myActiveLanes.push_back(i);
            }
            lu.amActive = true;
        }
    }
}
#endif
//Mahima
void
MSEdgeControl::patchActiveLanesGlobalCurrentStep(lane_info* lane_arr) {
   
   for (MSEdge* e : myEdges) {
        e->inferEdgeType();
        const std::vector<MSLane*>& lanes = e->getLanes();
        for (std::vector<MSLane*>::const_iterator j = lanes.begin(); j != lanes.end(); ++j) {
            LaneUsage& lu = myLanes[(*j)->getNumericalID()];
            (*j)->updateGlobalBrutto(0, 0);
            if(strcmp((*j)->getID().c_str(), "187955_0") == 0)
                std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  (*j)->getID() << "Brutto" << std::get<3>(lane_arr[(*j)->getNumericalID()]) << "\n";
            
            //std::get<3>(lane_arr[libsumo::Lane::getLaneNumericalID("187955_0")])
            // if the lane was inactive but is now...
        if (!lu.amActive && std::get<0>(lane_arr[(*j)->getNumericalID()]) > 0) {
            // ... add to active lanes and mark as such
            if (lu.haveNeighbors) {
                myActiveLanes.push_front(*j);
                //std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  (i)->getID() << "\n";
            } else {
                myActiveLanes.push_back(*j);
            }
            lu.amActive = true;
        }
        }
    }
   #ifdef close
    for (auto v : lane_arr)
    {
        //std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  v.first << "\n";
        //std::vector<std::tuple<std::string, double, double, double, double, double, bool>> vehp = v.second;
        std::vector<std::tuple<int, int, int, double>> &lanev = v.second;
        int lane_id = v.first;
        LaneUsage& lu = myLanes[v.first];
        MSLane* i = lu.lane;  
            
        // if the lane was inactive but is now...
        if (!lu.amActive && lanev.size() > 0) {
            // ... add to active lanes and mark as such
            if (lu.haveNeighbors) {
                myActiveLanes.push_front(i);
                //std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  (i)->getID() << "\n";
            } else {
                myActiveLanes.push_back(i);
            }
            lu.amActive = true;
        } /*else if(lu.amActive && vehp.size() == 0) {
            lu.amActive = false;
        }*/
    }
    #endif
}

void
MSEdgeControl::ReadBruttoGlobal(lane_info* lane_arr) {
    for (std::list<MSLane*>::iterator l = myActiveLanes.begin(); l != myActiveLanes.end();) {
        (*l)->updateGlobalBrutto(0, 0);
        (*l)->updateGlobalBrutto(std::get<3>(lane_arr[(*l)->getNumericalID()]), 1);
        std::cout << SIMTIME << " ReadBruttoGlobal lANE=" <<  (*l)->getID() << "Brutto: " << std::get<3>(lane_arr[(*l)->getNumericalID()]) << "\n";
        ++l;
    }  
   /* for (auto v : lane_arr)
    {
        //std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  v.first << "\n";
        std::vector<std::tuple<int, int, int, double>> lanev = v.second;
        int lane_id = v.first;
        LaneUsage& lu = myLanes[v.first];
        MSLane* i = lu.lane;  
        for(auto jj=0; jj<lanev.size(); ++jj)
        {
            i->updateGlobalBrutto(std::get<3>(lanev[jj]), 1);
        }
    }*/
}                        

void
MSEdgeControl::patchActiveLanes() {
    for (std::set<MSLane*, ComparatorNumericalIdLess>::iterator i = myChangedStateLanes.begin(); i != myChangedStateLanes.end(); ++i) {
        LaneUsage& lu = myLanes[(*i)->getNumericalID()];
        // if the lane was inactive but is now...
        if (!lu.amActive && (*i)->getVehicleNumber() > 0) {
            // ... add to active lanes and mark as such
            if (lu.haveNeighbors) {
                myActiveLanes.push_front(*i);
            } else {
                myActiveLanes.push_back(*i);
            }
            lu.amActive = true;
        }
    }
    myChangedStateLanes.clear();
}

void
MSEdgeControl::writeBruttoGlobal(lane_info* lane_arr) {
    /*for (std::list<MSLane*>::iterator l = myActiveLanes.begin(); l != myActiveLanes.end();) {
        (*l)->updateGlobalBrutto(0, 0);
        ++l;
    }*/
    //std::cout << SIMTIME << "done 1" << "\n";
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) { 
        if(strcmp((*i)->getID().c_str(), "6629_1") == 0)
            std::cout << SIMTIME << " writeBruttoGlobal lANE=" << (*i)->getID() << "std:get<4>" <<std::get<4>(lane_arr[(*i)->getNumericalID()]) << "std:get<3>" <<std::get<3>(lane_arr[(*i)->getNumericalID()]) << "active" << myLanes[(*i)->getNumericalID()].amActive << "brutto" << (*i)->getBruttoVehLenSum()<< "\n";
        if (myLanes[(*i)->getNumericalID()].amActive) {
           // if(!std::get<4>(lane_arr[(*i)->getNumericalID()]))
           // {
                //if(std::get<5>(lane_arr[(*i)->getNumericalID()]))
                //    std::get<3>(lane_arr[(*i)->getNumericalID()]) = (*i)->getBruttoVehLenSum() + std::get<0>(lane_arr[(*i)->getNumericalID()]);
                //else
            ///if(std::get<0>(lane_arr[(*i)->getNumericalID()]))
            ///    std::get<3>(lane_arr[(*i)->getNumericalID()]) = std::get<0>(lane_arr[(*i)->getNumericalID()]);
            ///else
                std::get<3>(lane_arr[(*i)->getNumericalID()]) = (*i)->getBruttoVehLenSum();
        }

        ++i;
        
    }
}

#ifdef close
//Mahima
void
MSEdgeControl::planGlobalMovements(SUMOTime t, std::tuple<int, int, int, double> lane_arr) {
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
        const int vehNum = (*i)->getVehicleNumber();
        if (vehNum == 0) {
            myLanes[(*i)->getNumericalID()].amActive = false;
            i = myActiveLanes.erase(i);
        } else {
        (*i)->planMovementsGlobal(t, lane_arr);
        
            ++i;
        }       
    }

    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
    //Update global brutto
        for (auto &v : lane_arr)
        {
            //std::cout << SIMTIME << " patchActiveLanesGlobal lANE=" <<  v.first << "\n";
            std::vector<std::tuple<int, int, int, double>> &lanev = v.second;
            int lane_id = v.first;
            LaneUsage& lu = myLanes[v.first];
            MSLane* l = lu.lane;
            if(lane_id == (*i)->getNumericalID())
            {
                for(auto jj=0; jj<lanev.size(); ++jj)
                {
                    std::get<3>(lanev[jj]) = (*i)->getBruttoVehLenSum();
                }
            }        
        }
        ++i;
    }    
}
#endif
//Mahima
void
MSEdgeControl::ResetGlobals() {
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
        (*i)->ResetGlobalvector();
            ++i;    
    }
}

void
MSEdgeControl::planGlobalMovements(SUMOTime t, lane_info* lane_arr, veh_info* veh_arr)
{
    #ifdef PARALLEL_STOPWATCH
    myStopWatch[0].start();
#endif
#ifdef THREAD_POOL
    std::vector<std::future<void>> results;
#endif
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
        const int vehNum = (*i)->getVehicleNumber();
        if (vehNum == 0) {
            myLanes[(*i)->getNumericalID()].amActive = false;
            //(*i)->updateGlobalBrutto(0, 0);
            i = myActiveLanes.erase(i);
        } else {
#ifdef THREAD_POOL
            if (MSGlobals::gNumSimThreads > 1) {
                results.push_back(myThreadPool.executeAsync([i, t](int) {
                    (*i)->planMovements(t);
                }, (*i)->getRNGIndex() % MSGlobals::gNumSimThreads));
                ++i;
                continue;
            }
#else
#ifdef HAVE_FOX
            if (MSGlobals::gNumSimThreads > 1) {
                myThreadPool.add((*i)->getPlanMoveTask(t), (*i)->getRNGIndex() % myThreadPool.size());
                ++i;
                continue;
            }
#endif
#endif
            //std::cout << SIMTIME << " lANE=" << (*i)->getID() << "\n";
            (*i)->planMovementsGlobal(t, lane_arr, veh_arr);
            //std::get<3>(lane_arr[(*i)->getNumericalID()]) = (*i)->getBruttoVehLenSum();
            ++i;
        }
    }
#ifdef THREAD_POOL
    for (auto& r : results) {
        r.wait();
    }
#else
#ifdef HAVE_FOX
    if (MSGlobals::gNumSimThreads > 1) {
        myThreadPool.waitAll(false);
    }
#endif
#endif
#ifdef PARALLEL_STOPWATCH
    myStopWatch[0].stop();
#endif
}

void
MSEdgeControl::planMovements(SUMOTime t) {
#ifdef PARALLEL_STOPWATCH
    myStopWatch[0].start();
#endif
#ifdef THREAD_POOL
    std::vector<std::future<void>> results;
#endif
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
        const int vehNum = (*i)->getVehicleNumber();
        if (vehNum == 0) {
            myLanes[(*i)->getNumericalID()].amActive = false;
            //(*i)->updateGlobalBrutto(0, 0);
            i = myActiveLanes.erase(i);
        } else {
#ifdef THREAD_POOL
            if (MSGlobals::gNumSimThreads > 1) {
                results.push_back(myThreadPool.executeAsync([i, t](int) {
                    (*i)->planMovements(t);
                }, (*i)->getRNGIndex() % MSGlobals::gNumSimThreads));
                ++i;
                continue;
            }
#else
#ifdef HAVE_FOX
            if (MSGlobals::gNumSimThreads > 1) {
                myThreadPool.add((*i)->getPlanMoveTask(t), (*i)->getRNGIndex() % myThreadPool.size());
                ++i;
                continue;
            }
#endif
#endif
            //std::cout << SIMTIME << " lANE=" << (*i)->getID() << "\n";
            (*i)->planMovements(t);
            ++i;
        }
    }
#ifdef THREAD_POOL
    for (auto& r : results) {
        r.wait();
    }
#else
#ifdef HAVE_FOX
    if (MSGlobals::gNumSimThreads > 1) {
        myThreadPool.waitAll(false);
    }
#endif
#endif
#ifdef PARALLEL_STOPWATCH
    myStopWatch[0].stop();
#endif
}


void
MSEdgeControl::setJunctionApproaches(SUMOTime t) {
    for (MSLane* const lane : myActiveLanes) {
        lane->setJunctionApproaches(t);
    }
}


void
MSEdgeControl::executeMovements(SUMOTime t) {
#ifdef PARALLEL_STOPWATCH
    myStopWatch[1].start();
#endif
    std::vector<MSLane*> wasActive(myActiveLanes.begin(), myActiveLanes.end());
    myWithVehicles2Integrate.clear();
#ifdef PARALLEL_EXEC_MOVE
#ifdef THREAD_POOL
    if (MSGlobals::gNumSimThreads > 1) {
        for (MSLane* const lane : myActiveLanes) {
            myThreadPool.executeAsync([lane, t](int) {
                lane->executeMovements(t);
            }, lane->getRNGIndex() % MSGlobals::gNumSimThreads);
        }
        myThreadPool.waitAll();
    }
#else
#ifdef HAVE_FOX
    if (MSGlobals::gNumSimThreads > 1) {
        for (MSLane* const lane : myActiveLanes) {
            myThreadPool.add(lane->getExecuteMoveTask(t), lane->getRNGIndex() % myThreadPool.size());
        }
        myThreadPool.waitAll(false);
    }
#endif
#endif
#endif
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
        if (
#ifdef PARALLEL_EXEC_MOVE
            MSGlobals::gNumSimThreads <= 1 &&
#endif
            (*i)->getVehicleNumber() > 0) {
            (*i)->executeMovements(t);
        }
        if ((*i)->getVehicleNumber() == 0) {
            myLanes[(*i)->getNumericalID()].amActive = false;
            i = myActiveLanes.erase(i);
        } else {
            ++i;
        }
    }
    //Mahima: empty lane_arr bcoz not being used in sumo
    lane_info* lane_arr;
    for (MSLane* lane : wasActive) {
        lane->updateLengthSum(lane_arr);
    }
    // arrived vehicles should not influence lane changing
    MSNet::getInstance()->getVehicleControl().removePending();
    std::vector<MSLane*>& toIntegrate = myWithVehicles2Integrate.getContainer();
    std::sort(toIntegrate.begin(), toIntegrate.end(), ComparatorIdLess());
    /// @todo: sorting only needed to account for lane-ordering dependencies.
    //This should disappear when parallelization is working. Until then it would
    //be better to use ComparatorNumericalIdLess instead of ComparatorIdLess
    myWithVehicles2Integrate.unlock();
    for (MSLane* const lane : toIntegrate) {
        const bool wasInactive = lane->getVehicleNumber() == 0;
        //Mahima
        lane->integrateNewVehicles(lane_arr);
        if (wasInactive && lane->getVehicleNumber() > 0) {
            LaneUsage& lu = myLanes[lane->getNumericalID()];
            if (!lu.amActive) {
                if (lu.haveNeighbors) {
                    myActiveLanes.push_front(lane);
                } else {
                    myActiveLanes.push_back(lane);
                }
                lu.amActive = true;
            }
        }
    }
#ifdef PARALLEL_STOPWATCH
    myStopWatch[1].stop();
#endif
}

void
MSEdgeControl::executeMovementsGlobal(SUMOTime t, lane_info* lane_arr) {
#ifdef PARALLEL_STOPWATCH
    myStopWatch[1].start();
#endif
    std::vector<MSLane*> wasActive(myActiveLanes.begin(), myActiveLanes.end());
    myWithVehicles2Integrate.clear();
#ifdef PARALLEL_EXEC_MOVE
#ifdef THREAD_POOL
    if (MSGlobals::gNumSimThreads > 1) {
        for (MSLane* const lane : myActiveLanes) {
            myThreadPool.executeAsync([lane, t](int) {
                lane->executeMovements(t);
            }, lane->getRNGIndex() % MSGlobals::gNumSimThreads);
        }
        myThreadPool.waitAll();
    }
#else
#ifdef HAVE_FOX
    if (MSGlobals::gNumSimThreads > 1) {
        for (MSLane* const lane : myActiveLanes) {
            myThreadPool.add(lane->getExecuteMoveTask(t), lane->getRNGIndex() % myThreadPool.size());
        }
        myThreadPool.waitAll(false);
    }
#endif
#endif
#endif
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
        if (
#ifdef PARALLEL_EXEC_MOVE
            MSGlobals::gNumSimThreads <= 1 &&
#endif
            (*i)->getVehicleNumber() > 0) {
            (*i)->executeMovementsGlobal(t, lane_arr);
        }
        if ((*i)->getVehicleNumber() == 0) {
            myLanes[(*i)->getNumericalID()].amActive = false;
            i = myActiveLanes.erase(i);
        } else {
            ++i;
        }
    }
    
    for (MSLane* lane : wasActive) {
        lane->updateLengthSum(lane_arr);
    }

    // arrived vehicles should not influence lane changing
    MSNet::getInstance()->getVehicleControl().removePending();
       
    std::vector<MSLane*>& toIntegrate = myWithVehicles2Integrate.getContainer();
    std::sort(toIntegrate.begin(), toIntegrate.end(), ComparatorIdLess());
    /// @todo: sorting only needed to account for lane-ordering dependencies.
    //This should disappear when parallelization is working. Until then it would
    //be better to use ComparatorNumericalIdLess instead of ComparatorIdLess
    myWithVehicles2Integrate.unlock();
    for (MSLane* const lane : toIntegrate) {
        const bool wasInactive = lane->getVehicleNumber() == 0;
        //Mahima
        lane->integrateNewVehicles(lane_arr);
        ///if(!std::get<4>(lane_arr[lane->getNumericalID()]))
        ///    std::get<3>(lane_arr[lane->getNumericalID()]) = lane->getBruttoVehLenSum() + std::get<0>(lane_arr[lane->getNumericalID()]);
          
        if (wasInactive && lane->getVehicleNumber() > 0) {
            LaneUsage& lu = myLanes[lane->getNumericalID()];
            if (!lu.amActive) {
                if (lu.haveNeighbors) {
                    myActiveLanes.push_front(lane);
                } else {
                    myActiveLanes.push_back(lane);
                }
                lu.amActive = true;
            }
        }
    }
#ifdef PARALLEL_STOPWATCH
    myStopWatch[1].stop();
#endif
}

void
MSEdgeControl::changeLanes(const SUMOTime t, lane_info* lane_arr) {
    std::vector<MSLane*> toAdd;
#ifdef PARALLEL_CHANGE_LANES
    std::vector<const MSEdge*> recheckLaneUsage;
#endif
    MSGlobals::gComputeLC = true;
    for (const MSLane* const l : myActiveLanes) {
        if (myLanes[l->getNumericalID()].haveNeighbors) {
            const MSEdge& edge = l->getEdge();
            if (myLastLaneChange[edge.getNumericalID()] != t) {
                myLastLaneChange[edge.getNumericalID()] = t;
#ifdef PARALLEL_CHANGE_LANES
                if (MSGlobals::gNumSimThreads > 1) {
                    MSLane* lane = edge.getLanes()[0];
                    myThreadPool.add(lane->getLaneChangeTask(t), lane->getRNGIndex() % myThreadPool.size());
                    recheckLaneUsage.push_back(&edge);
                } else {
#endif
                    edge.changeLanes(t, lane_arr);
                    for (MSLane* const lane : edge.getLanes()) {
                        LaneUsage& lu = myLanes[lane->getNumericalID()];
                        //if ((*i)->getID() == "disabled") {
                        //    std::cout << SIMTIME << " vehicles=" << toString((*i)->getVehiclesSecure()) << "\n";
                        //    (*i)->releaseVehicles();
                        //}
                        //Mahima
                        if(SIMTIME == 279.50) {
                        if (strcmp(lane->getID().c_str(), "18831_0") == 0) 
                            std::cout << SIMTIME << " 18831_0 vehicles=" << toString(lane->getVehiclesSecure()) << "\n";
                        if (strcmp(lane->getID().c_str(), "18831_1") == 0) 
                            std::cout << SIMTIME << " 18831_1 vehicles=" << toString(lane->getVehiclesSecure()) << "\n";
                        }
                #ifdef QSUMO_GLOBAL   
                        //lane->updateGlobalBrutto(lane->getBruttoVehLenSum(), 1);
                        /*if(std::get<5>(lane_arr[lane->getNumericalID()]))
                            std::get<3>(lane_arr[lane->getNumericalID()]) = std::get<0>(lane_arr[lane->getNumericalID()])+lane->getBruttoVehLenSum();
                        else if(!std::get<4>(lane_arr[lane->getNumericalID()]))
                            std::get<3>(lane_arr[lane->getNumericalID()]) = lane->getBruttoVehLenSum();
                        *///Mahima: Read global shared Brutto
                        //lane->updateGlobalBrutto(std::get<3>(lane_arr[lane->getNumericalID()]), 1);   
                        ///if(std::get<0>(lane_arr[lane->getNumericalID()]))
                        ///    std::get<3>(lane_arr[lane->getNumericalID()]) = std::get<0>(lane_arr[lane->getNumericalID()]);
                        ///else
                            ///std::get<3>(lane_arr[lane->getNumericalID()]) = lane->getBruttoVehLenSum();
                #endif        
                        if (lane->getVehicleNumber() > 0 && !lu.amActive) {
                            toAdd.push_back(lane);
                            lu.amActive = true;
                        }
                        if (MSGlobals::gLateralResolution > 0) {
                            lane->sortManeuverReservations();
                        }
                    }
                    

#ifdef PARALLEL_CHANGE_LANES
                }
#endif
            }
            
        } else {
            break;
        }
       
    }

#ifdef PARALLEL_CHANGE_LANES
    if (MSGlobals::gNumSimThreads > 1) {
        myThreadPool.waitAll(false);
        for (const MSEdge* e : recheckLaneUsage) {
            for (MSLane* const l : e->getLanes()) {
                LaneUsage& lu = myLanes[l->getNumericalID()];
                if (l->getVehicleNumber() > 0 && !lu.amActive) {
                    toAdd.push_back(l);
                    lu.amActive = true;
                }
                if (MSGlobals::gLateralResolution > 0) {
                    l->sortManeuverReservations();
                }
            }
        }
    }
#endif

    MSGlobals::gComputeLC = false;
    for (std::vector<MSLane*>::iterator i = toAdd.begin(); i != toAdd.end(); ++i) {
        myActiveLanes.push_front(*i);
    }
}


void
MSEdgeControl::detectCollisions(SUMOTime timestep, const std::string& stage, lane_info* lane_arr) {
    // Detections is made by the edge's lanes, therefore hand over.
    for (MSLane* lane : myActiveLanes) {
        if (lane->needsCollisionCheck()) {
            lane->detectCollisions(timestep, stage, lane_arr);
        }
    }
    if (myInactiveCheckCollisions.size() > 0) {
        for (MSLane* lane : myInactiveCheckCollisions.getContainer()) {
            lane->detectCollisions(timestep, stage, lane_arr);
        }
        myInactiveCheckCollisions.clear();
        myInactiveCheckCollisions.unlock();
    }
}


void
MSEdgeControl::gotActive(MSLane* l) {
    myChangedStateLanes.insert(l);
}

void
MSEdgeControl::checkCollisionForInactive(MSLane* l) {
    myInactiveCheckCollisions.insert(l);
}

void
MSEdgeControl::setAdditionalRestrictions() {
    for (MSEdge* e : myEdges) {
        e->inferEdgeType();
        const std::vector<MSLane*>& lanes = e->getLanes();
        for (std::vector<MSLane*>::const_iterator j = lanes.begin(); j != lanes.end(); ++j) {
            (*j)->initRestrictions();
        }
    }
}

void
MSEdgeControl::setMesoTypes() {
    for (MSEdge* edge : myEdges) {
        edge->updateMesoType();
    }
}

void
MSEdgeControl::saveState(OutputDevice& out) {
    out.openTag(SUMO_TAG_EDGECONTROL);
    out.writeAttr(SUMO_ATTR_LANES, myActiveLanes);
    out.closeTag();
}

/****************************************************************************/
