#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
//#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"
#include <ompl/base/objectives/StateCostIntegralObjective.h>

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/BITstar.h>
#include "ompl/geometric/PathGeometric.h"

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>

#include <boost/filesystem.hpp>
#include <ompl/util/PPM.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
/// @cond IGNORE
// MUST if load PPM file, replace to RGB Color
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si_) :
        ob::StateValidityChecker(si_) {
        }

    bool isValid(const ob::State* state) const override
    {
        try
        {
            const int w = (int)state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            const int h = (int)state->as<ob::RealVectorStateSpace::StateType>()->values[1];

            const ompl::PPM ppm = si_->getSpacePPM();

            const ompl::PPM::Color &c = ppm.getPixel(h,w);
            
            return c.red > 127 && c.green > 127 && c.blue > 127;
        }
        catch(const std::exception& e)
        {
            OMPL_ERROR(e.what());
            OMPL_ERROR("is catched in isValid");

        }
        return false;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const override
    {        
        try
        {
            // We know we're working with a RealVectorStateSpace in this
            // example, so we downcast state into the specific type.
            const auto* state2D =
                state->as<ob::RealVectorStateSpace::StateType>();

            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            const ompl::PPM ppm = si_->getSpacePPM();

            const ompl::PPM::Color &c = ppm.getPixel(y, x);
            if (c.red > 150 && c.green > 150 && c.blue > 150) 
                return 10;    
            else
                return 0;
        }
        catch(const std::exception& e)
        {
            OMPL_ERROR(e.what());
            OMPL_ERROR("is catched in clearance");

        }
        return 0;
    }
};

/**
 * @brief optimal objective
 */

class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si_) :
        ob::StateCostIntegralObjective(si_, true)
    {
    }

    // Our requirement is to maximize path clearance from obstacles
    ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(si_->getStateValidityChecker()->clearance(s));
    }
};

class StabilityObjective : public ob::StateCostIntegralObjective
{
public:
    StabilityObjective(const ob::SpaceInformationPtr& si_) :
        ob::StateCostIntegralObjective(si_,true)
    {    
    }

     // Our requirement is to maximize path clearance from obstacles
    ob::Cost stateCost(const ob::State* s) const override
    {
        try
        {
            const int w = (int)s->as<ob::RealVectorStateSpace::StateType>()->values[0];
            const int h = (int)s->as<ob::RealVectorStateSpace::StateType>()->values[1];

            const ompl::PPM ppm = si_->getSpacePPM();

            const ompl::PPM::Color &c = ppm.getPixel(h,w);
            
            if (c.red > 254 && c.green > 254 && c.blue > 168)
                return ob::Cost(0);
            return ob::Cost(1000);
        }
        catch(const std::exception& e)
        {
            OMPL_ERROR("In StabilityObjective.\n%s", e.what());
        }
        return ob::Cost(0);
    }
};
class tBITEnvironment
{
public:
    tBITEnvironment(const char *ppm_file) //, bool use_deterministic_sampling = false)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch (ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            // set space
            auto space(std::make_shared<ob::RealVectorStateSpace>());
            space->addDimension(0.0, ppm_.getWidth());
            space->addDimension(0.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            // Construct a space information instance for this state space
            si_ = std::make_shared<ob::SpaceInformation>(space);

            si_->setSpacePPM(ppm_);

            // set state validity checking for this space
            si_->setStateValidityChecker(std::make_shared<ValidityChecker>(si_));
            si_->setup();

            // set problem definition
            pdef_ = std::make_shared<ob::ProblemDefinition>(si_);

            // set Optimization Objective 
            auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si_));
            auto clearObj(std::make_shared<ClearanceObjective>(si_));
            auto stableObj(std::make_shared<StabilityObjective>(si_));
            auto opt(std::make_shared<ob::MultiOptimizationObjective>(si_));
            opt->addObjective(clearObj, 1.0);
            opt->addObjective(stableObj, 1.0);
            opt->addObjective(lengthObj, 10.0);
            opt->setCostToGoHeuristic(&ob::goalRegionCostToGo);
            pdef_->setOptimizationObjective(ob::OptimizationObjectivePtr(opt));

            //set planner
            optimizingPlanner_ = std::make_shared<og::BITstar>(si_);
            // Set the problem instance for our planner to solve
            optimizingPlanner_->setProblemDefinition(pdef_);
            optimizingPlanner_->setup();
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        ob::StateSpacePtr space = si_->getStateSpace();
        // Set our robot's starting state 
        ob::ScopedState<> start(space);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_row; //0.0
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_col; //0.0

        // Set our robot's goal state 
        ob::ScopedState<> goal(space);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_row;//1.0;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_col;//1.0;

        // Set the start and goal states
        pdef_->setStartAndGoalStates(start, goal,10);

        // attempt to solve the planning problem in the given runtime
        ob::PlannerStatus solved = optimizingPlanner_->solve(10000);

        if (solved)
        {
            // Output the length of the path found
            std::cout
                << optimizingPlanner_->getName()
                << " found a solution of length "
                << pdef_->getSolutionPath()->length()
                << std::endl;

            return true;
        }
        else
        {
            std::cout << "No solution found." << std::endl;
            return false;
        }
    }

    void recordSolution()
    {
        if (!pdef_ || !pdef_->hasSolution())
            return;
        std::ofstream outFile(outputFile_.c_str());
        const ob::PathPtr &pp = pdef_->getSolutionPath();
        //if (pp)
        og::PathGeometric &p = static_cast<og::PathGeometric &>(*pp);
        p.interpolate();
        for (std::size_t i = 0; i < p.getStateCount(); ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h =
                std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            // Record Solution Path as Metrix
            outFile << w << " " << h << std::endl;
            // Record Solution Path as Pixel
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
        outFile <<std::endl;
        outFile.close();
    }
    
    void save(const char *filename)
    {
        if (!pdef_)
        {
            return;
        }
        ppm_.saveFile(filename);
    }

private:
    ob::ProblemDefinitionPtr pdef_;
    ob::SpaceInformationPtr si_;
    ob::PlannerPtr optimizingPlanner_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
    std::string outputFile_;
};
// Parse the command-line arguments
/** Parse the command line arguments into a string for an output file and the planner/optimization types */

int main(/*int argc, char** argv*/)
{
    unsigned int start_row = 1295;//std::stoul(argv[2],nullptr,16);
    unsigned int start_col = 550;//std::stoul(argv[3],nullptr,16);
    unsigned int goal_row = 325;//std::stoul(argv[4],nullptr,16);
    unsigned int goal_col = 975;//std::stoul(argv[5],nullptr,16);

    ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);

    boost::filesystem::path path(TEST_RESOURCES_DIR);
    tBITEnvironment env((path / "test.ppm").string().c_str());
        // MUST add start and goal
        //(1741,803,826,270)
    if (env.plan(start_row, start_col, goal_row, goal_col))
    {
        env.recordSolution();
        OMPL_INFORM((path /  "result_pmu_route++2.ppm").string().c_str());
        env.save((path /  "result_pmu_route++2.ppm").string().c_str());
        // Return with success
        return 0;
    }
    // Return with error
    return -1;
}
/// @endcond
