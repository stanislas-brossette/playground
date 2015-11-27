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
  std::ofstream dirLogFile;
  dirLogFile.open("directionsLogPlanContact.txt");
  dirLogFile << "================================" << std::endl;
  //srand((unsigned int) time(0));
  ProblemConfig config("../../configs/statisticsLeaningOnSpherePlanWrist.yml");
  config.loadFile("../../configs/hrp2_drc_no_hands.yml");
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
  for(int i = 0; i < nTest; ++i)
  {
    std::cerr << "Test: " << i << std::endl;
    dir = Eigen::Vector3d::Random();
    dir.normalize();
    dirLogFile << "dir: " << dir.transpose();

    PostureProblem<solver_t::problem_t> pP({robot_dir + config.get("Robot.urdf")});

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

    //PostureGoal_On_Robot* pZOR =
      //obot::makeUNCHECKEDDescriptiveWrapper(&pZ, *r.getManifold());
    //pP.problemFactory.setObjective(*pZOR, *r.getManifold());
    //Expr costFunc
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

    //ContactParamSurfSphereCstr rHandSphere(r, rHandFrame, mySphere, frictionLimit);
    ContactPlanSphereCstr rHandSphere(r, RHandContactSurf, mySphere, frictionLimit);

    pP.addContactStab(lFootFloor);
    pP.addContactStab(rFootFloor);
    pP.addContactStab(rHandSphere);

    //r.getManifold()->display();

    roboptim::ProblemOnManifold<solver_t::problem_t>* myProblem = pP.getProblem();

   //myProblem->getManifold().display();

  //for(int i = 0; i < r.getMultiBody().nrBodies(); ++i)
    //{
      //std::cout << i << ": " << r.getMultiBody().body(i).name() << std::endl;
    //}

    Eigen::VectorXd input(myProblem->getManifold().getZero().value());

    //std::cout << "input.size() = \n" << input.size() << std::endl;
    input.head(7) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.76;
    hrp2DRC_nohands::getHalfSitting(input.segment(7,32));
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
    //std::cout << "input = \n" << input.transpose() << std::endl;

    myProblem->getManifold().forceOnM(input, input);
    myProblem->startingPoint() = input;

    roboptim::SolverFactory<solver_t> factory ("pgsolver_d", *myProblem);
    solver_t& solver = factory ();
    solver.setIterationCallback(callbackMngr.callback());
    //solver.parameters()["pgsolver.finiteDiffCheck"].value = true;
    //solver.parameters()["pgsolver.rho0"].value = 0.4;
    solver.parameters()["pgsolver.rhoMax"].value = config.get("pgsolver.rhoMax").asDouble();
    solver.parameters()["pgsolver.maxIter"].value = config.get("pgsolver.maxIter").asInt();
    solver.parameters()["pgsolver.VERBOSE"].value = config.get("pgsolver.VERBOSE").asInt();
    solver.parameters()["pgsolver.VERBOSE_FILTER"].value = config.get("pgsolver.VERBOSE_FILTER").asBool();
    //solver.parameters()["pgsolver.maxIter"].value = 2000;
    //solver.parameters()["pgsolver.VERBOSE"].value = 1;
    solver.parameters()["pgsolver.hessianUpdateMethod"].value = config.get("pgsolver.hessianUpdateMethod");

    res.push_back(solver.minimum());

    switch (res.back().which ())
    {
      case solver_t::SOLVER_VALUE:
      {
        // Get the result.
        roboptim::Result& result = boost::get<roboptim::Result> (res.back());
        dirLogFile << " res: " << result.value << std::endl;

        // DO THINGS WITH YOUR RESULT HERE, LIKE PRINTING IT TO COUT

        //std::cout << result << std::endl;
        break;
      }
 
      case solver_t::SOLVER_VALUE_WARNINGS:
      {
        // Get the result.
        roboptim::ResultWithWarnings& result = boost::get<roboptim::ResultWithWarnings> (res.back());
        dirLogFile << " WARNINGres: " << result.value << std::endl;

        // DO THINGS WITH YOUR RESULT HERE, LIKE PRINTING IT TO COUT
        // BEWARE, THERE ARE WARNINGS

        //std::cout << result << std::endl;
        break;
      }
 
      case solver_t::SOLVER_NO_SOLUTION:
      case solver_t::SOLVER_ERROR:
      {
        dirLogFile << " FAIL" << std::endl;
      }
    }
    //std::cout << "res: " << res << std::endl;

  //roboptim::Result result = boost::get<roboptim::Result>(res);

  // TODO: extract the solution from the solver
  //r.setConfig(result.x.head<49>(), true);
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

      rHandSphere.addToRviz(&rvizBroadcaster);

      CoordinatesPtr coordCoM(new RCoM(r));
      RotationPtr id3(Eigen::Matrix3d::Identity());
      Frame CoMFrame(r.getRefFrame().getWorld(), id3, coordCoM);
      rvizBroadcaster.addFrame(CoMFrame);

      Wrench weight(CoMFrame,Eigen::Vector3d(0, 0, -9.81*r.getMass()));
      rvizBroadcaster.addForce(weight);
      for (size_t i = 0; i < pP.stabAccNewton->externalForces().size(); ++i)
      {
        rvizBroadcaster.addForce(pP.stabAccNewton->externalForces()[i]);
      }

      r.getConfig().zero(r.getMultiBody());
      rvizBroadcaster.setStanceUpdateCallback([&pP](const Eigen::VectorXd& newValues){pP.updateVariables(newValues);}).setFilename("touchASphereAndPointAsFarAsPossible");
      rvizBroadcaster.broadcastToRViz();
    }
  }
  dirLogFile.close();
}

