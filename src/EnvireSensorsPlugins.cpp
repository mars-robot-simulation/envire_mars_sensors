/**
 * \file EnvireSensorsPlugins.cpp
 * \author Malte Langosz
 *
 */

#include "EnvireSensorsPlugins.hpp"

#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_interfaces/sim/LoadSceneInterface.h>

#include <mars_interfaces/Logging.hpp>
#include <mars_interfaces/MARSDefs.h>

#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_interfaces/sim/SensorManagerInterface.h>


namespace mars
{
    namespace envire_sensors
    {
        using namespace interfaces;
        using namespace utils;

        EnvireSensorsPlugins::EnvireSensorsPlugins(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface{theManager}
        {
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::sensors::CameraSensor>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::sensors::RaySensor>>::subscribe(ControlCenter::envireGraph.get());

            sim = libManager->getLibraryAs<SimulatorInterface>("mars_core");
        }

        EnvireSensorsPlugins::~EnvireSensorsPlugins()
        {
            if (sim)
            {
                // TODO: This would release the last reference to mars_core. This somehow results in a segmentation fault in mars_ode_physics::Joint::~Joint when calling dJointDestroy.
                // libManager->releaseLibrary("mars_core");
                // sim = nullptr;
            }
        }

        // TODO: this should be moved out from here
        std::shared_ptr<SubControlCenter> EnvireSensorsPlugins::getControlCenter(envire::core::FrameId frame)
        {
            // search for physics interface in graph
            bool done = false;
            while(!done)
            {
                const envire::core::GraphTraits::vertex_descriptor vertex = ControlCenter::envireGraph->vertex(frame);
                envire::core::GraphTraits::vertex_descriptor parentVertex = ControlCenter::graphTreeView->tree[vertex].parent;
                // todo: check if this check is correct
                if(parentVertex)
                {
                    frame = ControlCenter::envireGraph->getFrameId(parentVertex);
                    try
                    {
                        using SubControlItem = envire::core::Item<std::shared_ptr<SubControlCenter>>;
                        const auto& it = ControlCenter::envireGraph->getItem<SubControlItem>(frame);
                        return it->getData();
                    }
                    catch (...)
                    {
                    }
                }
                else
                {
                    done = true;
                }
            }
            return nullptr;
        }

        void EnvireSensorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::sensors::CameraSensor>>& e)
        {
            auto subControl = getControlCenter(e.frame);
            if (!subControl)
            {
                return;
            }

            auto& sensor = e.item->getData();
            auto config = sensor.getFullConfigMap();

            const auto drawID = sim->getControlCenter()->graphics->getDrawID(e.frame);
            if(drawID)
            {
                config["draw_id"] = drawID;

                const auto sensorID = sim->getControlCenter()->sensors->createAndAddSensor(&config);
                // TODO: temporarly add base sensor into the graph
                // we can replace it with the similar structure as DynamicObjectItem: BaseSensorItem
                std::shared_ptr<interfaces::BaseSensor> baseSensor{sim->getControlCenter()->sensors->getSimSensor(sensorID)};
                envire::core::Item<std::shared_ptr<interfaces::BaseSensor>>::Ptr baseSensorItemPtr{new envire::core::Item<std::shared_ptr<interfaces::BaseSensor>>(baseSensor)};
                ControlCenter::envireGraph->addItemToFrame(e.frame, baseSensorItemPtr);
            }
        }

        void EnvireSensorsPlugins::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::sensors::RaySensor>>& e)
        {
            auto subControl = getControlCenter(e.frame);
            if (!subControl)
            {
                return;
            }

            auto& sensor = e.item->getData();
            auto config = sensor.getFullConfigMap();

            // get parent smurf frame
            const auto& vertex = ControlCenter::envireGraph->vertex(e.frame);
            const auto& parentVertex = ControlCenter::graphTreeView->tree[vertex].parent;
            const auto& parentFrame = ControlCenter::envireGraph->getFrameId(parentVertex);

            bool found = false;
            config["groupName"] = "mars_sim";
            config["dataName"] = "Frames/"+parentFrame;
            const auto& tCenterToParent = ControlCenter::envireGraph->getTransform(SIM_CENTER_FRAME_NAME, parentFrame);
            vectorToConfigItem((&config["init_position"]), &(tCenterToParent.transform.translation));
            quaternionToConfigItem(&(config["init_orientation"]), &(tCenterToParent.transform.orientation));
            const auto& tParentToItem = ControlCenter::envireGraph->getTransform(parentFrame, e.frame);
            vectorToConfigItem(&(config["pos_offset"]), &(tParentToItem.transform.translation));
            quaternionToConfigItem(&(config["ori_offset"]), &(tParentToItem.transform.orientation));

            const auto sensorID = sim->getControlCenter()->sensors->createAndAddSensor(&config);
            // TODO: temporarly add base sensor into the graph
            // we can replace it with the similar structure as DynamicObjectItem: BaseSensorItem
            //std::shared_ptr<interfaces::BaseSensor> baseSensor;
            //baseSensor.reset(sim->getControlCenter()->sensors->createAndAddSensor(&config));
            //envire::core::Item<std::shared_ptr<interfaces::BaseSensor>>::Ptr sensorItemPtr(new envire::core::Item<std::shared_ptr<interfaces::BaseSensor>>(baseSensor));
            //ControlCenter::envireGraph->addItemToFrame(e.frame, sensorItemPtr);
        }
    } // end of namespace envire_sensors
} // end of namespace mars

DESTROY_LIB(mars::envire_sensors::EnvireSensorsPlugins);
CREATE_LIB(mars::envire_sensors::EnvireSensorsPlugins);
