#include <fstream>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <roboptim/core/solver-factory.hh>
#include <boost/mpl/list.hpp>

#include <manifolds/Manifold.h>
#include <manifolds/RealSpace.h>

#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/decorator/finite-difference-gradient.hh>
#include <roboptim/core/detail/structured-input.hh>

#include <RBDyn/MultiBodyConfig.h>

#include <roboptim/core/manifold-map/decorator/manifold-map.hh>
#include <roboptim/core/manifold-map/decorator/problem-on-manifold.hh>

#include <ProblemGenerator/PostureProblem.hh>
#include <ProblemGenerator/constraint/COMProjectionStabilityAccumulator.hh>
#include <ProblemGenerator/utils/RVizRobotBroadcaster.hh>

#include <ProblemGenerator/constraint/MatchPosCstr.hh>
#include <ProblemGenerator/constraint/MatchPosWithFuncCstr.hh>
#include <ProblemGenerator/constraint/ContactPlanSphereCstr.hh>
#include <ProblemGenerator/constraint/ContactParamSurfSphereCstr.hh>
#include <ProblemGenerator/constraint/FixedContactCstr.hh>
#include <ProblemGenerator/constraint/FixedStabilityContactCstr.hh>
#include <ProblemGenerator/constraint/FloatingPlanarContactCstr.hh>
#include <ProblemGenerator/constraint/FixedPlanarContactCstr.hh>

#include <ProblemGenerator/constraint/function/MatchRobotPointWithEnv.hh>
#include <ProblemGenerator/constraint/function/MatchRobotPointWithFunc.hh>
#include <ProblemGenerator/constraint/function/FixedOrientation.hh>
#include <ProblemGenerator/constraint/function/DistanceToPostureGoal.hh>
#include <ProblemGenerator/constraint/function/DistanceToPostureGoalAndPointAsFarAsPossible.hh>
#include <ProblemGenerator/constraint/function/FixInPlan.hh>

#include <ProblemGenerator/utils/defs.hh>
#include <ProblemGenerator/utils/hrp2drcPose.hh>
#include <ProblemGenerator/utils/callbackManager.hh>

#include <RobotFeatures/features/Expr.h>
#include <RobotFeatures/features/Operations.h>
#include <RobotFeatures/features/Frame.h>
#include <RobotFeatures/features/Point.h>
#include <RobotFeatures/features/Vector.h>


using namespace Eigen;

int main()
{
  //srand((unsigned int) time(0));
  ProblemConfig config("../../configs/statisticsLeaningOnSphereParamWrist.yml");
  config.loadFile("../../configs/hrp2_drc_no_hands.yml");
  std::ofstream dirLogFile;
  std::string dirLogFileName = config["dirLogFileName"];
  dirLogFile.open(dirLogFileName);
  dirLogFile << "================================" << std::endl;
  double frictionLimit = config.get("frictionLimit").asDouble();
  int nTest = config.get("nTest").asInt();
  std::string rHandContactSurf(config.get("rHandContactSurf"));
  std::vector<VectorXd> iterations;
  std::vector<VectorXd> finalIterations;
  iterations.clear();
  finalIterations.clear();
  callbackManager callbackMngr(&iterations);
  std::string robot_dir(config.get("Robot.dir"));

  Eigen::Vector3d dir;//(config.get("goalDir").asVector3d());
  std::vector<solver_t::result_t> res;
  for (int i = 0; i < nTest; ++i)
  {
    std::cerr << "Test: " << i << std::endl;
    dir = Eigen::Vector3d::Random();
    dir.normalize();

    PostureProblem<T> pP({robot_dir + config.get("Robot.urdf")});

    pP.setStabilityCriterion(Newton);

    rbf::Robot& r = *(pP.getRobot(0));
    RVizRobotBroadcaster rvizBroadcaster(&r, &finalIterations);

    r.loadSurfacesFromRSDF(robot_dir + config.get("Robot.lFootSurf"));
    r.loadSurfacesFromRSDF(robot_dir + config.get("Robot.rFootSurf"));
    r.loadSurfacesFromRSDF(robot_dir + config.get("Robot.rHandSurf"));
    r.loadSurfacesFromRSDF(robot_dir + config.get("Robot.lHandSurf"), true);

    rbf::Robot::Surface LHandFrontSurf(r.getSurface("LeftFingers"));
    rbf::Robot::Surface RHandContactSurf(r.getSurface(rHandContactSurf));

    Eigen::VectorXd goalPos(32);
    goalPos.setZero();
    hrp2DRC_nohands::getHalfSitting(goalPos);
    Point pLHand(r.getBodyFrame(r.getBodyId("LARM_LINK6")), Eigen::Vector3d(0, 0, 0));

    PostureGoalAndPointAsFarAsPossible pZ(static_cast<int>(r.getManifold()->representationDim()), r, goalPos, pLHand, dir);
    Point O(r.getRefFrame().getWorld(), Eigen::Vector3d(0,0,0));
    Vector dirVec(r.getRefFrame().getWorld(), dir);
    ScalarPtr distAlongDirection = (pLHand - O).dot(dirVec);

    PostureGoalAndPointAsFarAsPossible_On_Robot* posAndPoint =
      PostureGoalAndPointAsFarAsPossible_On_Robot::makeUNCHECKEDDescriptiveWrapper(
          &pZ, *r.getManifold());
    pP.problemFactory.setObjective(*posAndPoint, *r.getManifold());

    rbf::Robot::Surface LFullSoleSurf(r.getSurface("LFullSole"));
    rbf::Robot::Surface RFullSoleSurf(r.getSurface("RFullSole"));

    Point p1(r.getRefFrame().getWorld(), Vector3d(0, -0.15, 0));
    Point p2(r.getRefFrame().getWorld(), Vector3d(0, 0.15, 0));
    Frame groundFrame(r.getRefFrame().getWorld(), Matrix3d::Identity(), Vector3d::Zero());
    Frame groundRFootFrame(r.getRefFrame().getWorld(), Matrix3d::Identity(), p1.coord(r.getRefFrame().getWorld()));
    Frame groundLFootFrame(r.getRefFrame().getWorld(), Matrix3d::Identity(), p2.coord(r.getRefFrame().getWorld()));

    FixedPlanarContactCstr lFootFloor(LFullSoleSurf, r, groundLFootFrame, *(LFullSoleSurf.frame), frictionLimit);
    FixedPlanarContactCstr rFootFloor(RFullSoleSurf, r, groundRFootFrame, *(RFullSoleSurf.frame), frictionLimit);

    Sphere mySphere(config("mySphere"));

    //rHandFrame is the Zero Fraom of the param function
    Frame rHandFrame(*(RHandContactSurf.frame), Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));

    ContactParamSurfSphereCstr rHandParamSphere(r, rHandFrame, mySphere, frictionLimit);
    ContactPlanSphereCstr rHandPlanSphere(r, RHandContactSurf, mySphere, frictionLimit);

    pP.addContactStab(lFootFloor);
    pP.addContactStab(rFootFloor);

    bool paramContact = config["paramContact"].asBool();
    if(paramContact)
      pP.addContactStab(rHandParamSphere);
    else
      pP.addContactStab(rHandPlanSphere);

    roboptim::ProblemOnManifold<T>* myProblem = pP.getProblem();

    Eigen::VectorXd input(myProblem->getManifold().getZero().value());

    input.head(7) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.76;
    hrp2DRC_nohands::getHalfSitting(input.segment(7,32));
    if(paramContact)
      input.tail(input.size()-39) <<
          -1, 0, 0,
          0,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 10;
    else
      input.tail(input.size()-39) <<
          -1, 0, 0,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 80,
          0, 0, 10;
    //input(42) = config.get("valueR1").asDouble();
    //input.segment(4,32) += 0.01*Eigen::VectorXd::Random(32);

    myProblem->getManifold().forceOnM(input, input);
    myProblem->startingPoint() = input;

    roboptim::SolverFactory<solver_t> factory ("pgsolver_d", *myProblem);
    solver_t& solver = factory ();
    solver.setIterationCallback(callbackMngr.callback());
    solver.parameters()["pgsolver.rhoMax"].value = config.get("pgsolver.rhoMax").asDouble();
    solver.parameters()["pgsolver.maxIter"].value = config.get("pgsolver.maxIter").asInt();
    solver.parameters()["pgsolver.VERBOSE"].value = config.get("pgsolver.VERBOSE").asInt();
    solver.parameters()["pgsolver.VERBOSE_FILTER"].value = config.get("pgsolver.VERBOSE_FILTER").asBool();
    solver.parameters()["pgsolver.hessianUpdateMethod"].value = config.get("pgsolver.hessianUpdateMethod");

    res.push_back(solver.minimum());


    switch (res.back().which ())
    {
      case solver_t::SOLVER_VALUE:
      {
        // Get the result.
        roboptim::Result& result = boost::get<roboptim::Result> (res.back());
        dirLogFile << "SUCCESS: " ;
        //dirLogFile << " res: " << result.value << std::endl;
        r.setConfig(result.x.head<39>(), true);

        // DO THINGS WITH YOUR RESULT HERE, LIKE PRINTING IT TO COUT

        //std::cout << result << std::endl;
        break;
      }

      case solver_t::SOLVER_VALUE_WARNINGS:
      {
        // Get the result.
        roboptim::ResultWithWarnings& result = boost::get<roboptim::ResultWithWarnings> (res.back());
        dirLogFile << "WARNING: ";
        //dirLogFile << " WARNINGres: " << result.value << std::endl;
        r.setConfig(result.x.head<39>(), true);

        // DO THINGS WITH YOUR RESULT HERE, LIKE PRINTING IT TO COUT
        // BEWARE, THERE ARE WARNINGS

        //std::cout << result << std::endl;
        break;
      }

      case solver_t::SOLVER_NO_SOLUTION:
      case solver_t::SOLVER_ERROR:
      {
        dirLogFile << "FAIL: ";
      }
    }
    //std::cout << "res: " << res << std::endl;


  // TODO: extract the solution from the solver
    //dirLogFile << "dir: " << dir.transpose() << std::endl;
    //dirLogFile << "dist: " << distAlongDirection.value() << std::endl;
    dirLogFile << "dist*dir: " << distAlongDirection.value()*dir.transpose() << std::endl;
    finalIterations.push_back(iterations.back());

    if(i == nTest-1)
    {
      std::pair<Eigen::Vector3d, double> s = std::make_pair(mySphere.O_, mySphere.R_);
      //rvizBroadcaster.addPoint(p1);
      //rvizBroadcaster.addPoint(p2);
      rvizBroadcaster.addSphere(s);
      rvizBroadcaster.addFrame(groundFrame);
      rvizBroadcaster.addFrame(rHandFrame);
      rvizBroadcaster.addSurface(&(r.getSurface("LFullSole")));
      rvizBroadcaster.addSurface(&(r.getSurface("RFullSole")));
      rvizBroadcaster.addSurface(&(r.getSurface(rHandContactSurf)));

      if(paramContact)
        rHandParamSphere.addToRviz(&rvizBroadcaster);

      CoordinatesPtr coordCoM(new RCoM(r));
      RotationPtr id3(Eigen::Matrix3d::Identity());
      Frame CoMFrame(r.getRefFrame().getWorld(), id3, coordCoM);
      rvizBroadcaster.addFrame(CoMFrame);

      Wrench weight(CoMFrame,Eigen::Vector3d(0, 0, -9.81*r.getMass()));
      rvizBroadcaster.addForce(weight);

      for (auto extForces : pP.stabAccNewton->externalForces())
	{
	  for (size_t i = 0; i < extForces.second.size(); ++i)
	    {
	      rvizBroadcaster.addForce(extForces.second[i]);
	    }
	}

      r.getConfig().zero(r.getMultiBody());
      rvizBroadcaster.setStanceUpdateCallback([&pP](const Eigen::VectorXd& newValues){pP.updateVariables(newValues);}).setFilename("touchASphereAndPointAsFarAsPossible");
      rvizBroadcaster.broadcastToRViz();
    }
  }
  dirLogFile.close();
}
