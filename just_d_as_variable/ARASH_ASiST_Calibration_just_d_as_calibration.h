#pragma once

#include <iostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/optional.hpp>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


using namespace gtsam;
using namespace std;

/*
    //Arash_asist calibration factor//
    Measurements: point3 endeffector, phi, psi, d
    What we know: the forward kinematic of our robot
    Variables: alpha, theta, L
    E(q) = z - h(q) = Point3_GT - Point3_fk
*/

namespace gtsam
{  
    class ArashAsistFactor : public NoiseModelFactor1<double>
    {
        // Private member variables to store ground truth and calibration parameters
        private:
        gtsam::Vector3 groud_truth;
        double phi;
        double psi;
        double alpha;
        double theta;
        double L;


        public:
        // Constructor
        ArashAsistFactor(Key key1, double alpha_, double theta_, double L_, gtsam::Vector3 groud_truth_, double phi_, double psi_, const SharedNoiseModel &model) 
        : NoiseModelFactor1<double>(model, key1), groud_truth(groud_truth_), phi(phi_), psi(psi_), alpha(alpha_), theta(theta_), L(L_) {}

        // Evaluate the error
        Vector evaluateError(const double &d,
                             OptionalMatrixType H1) const override
        {
            // Forward kinematic model of robot which contain the variables to optimize (this is type of vector (x, y, z))
            gtsam::Vector3 task_point;
            task_point[0] = 0.1000e4 * L * sin(theta) - 0.1000e4 * d * (cos(alpha + psi) * sin(theta) + sin(alpha + psi) * cos(phi) * cos(theta));
            task_point[1] = -0.1000e4 * d * sin(alpha + psi) * sin(phi);
            task_point[2] = 0.1000e4 * L * cos(theta) - 0.1000e4 * d * (cos(alpha + psi) * cos(theta) - sin(alpha + psi) * cos(phi) * sin(theta));
            gtsam::Vector3 measurement_model(task_point[0], task_point[1], task_point[2]);
            if (H1)
            {   
                // model jacobian with respect to d (the forth optimization variable)
                gtsam::Vector3 derivative_wrt_d;
                derivative_wrt_d[0] = -0.1000e4 * cos(alpha + psi) * sin(theta) - 0.1000e4 * sin(alpha + psi) * cos(phi) * cos(theta);
                derivative_wrt_d[1] = -0.1000e4 * sin(alpha + psi) * sin(phi);
                derivative_wrt_d[2] = -0.1000e4 * cos(alpha + psi) * cos(theta) + 0.1000e4 * sin(alpha + psi) * cos(phi) * sin(theta);

                *H1 = (gtsam::Matrix(3, 1) << derivative_wrt_d[0], derivative_wrt_d[1], derivative_wrt_d[2]).finished();
            }

            // return the |h(q) - z| ----> |measurement_model - measurememnt|
            gtsam::Vector3 error = task_point - groud_truth;
            return error;
        }
    };
}

/*
    this is another custom factor which developed to including constains of variables
*/
namespace gtsam
{   
    class Inequality : public NoiseModelFactor1<double>
    {
        public:
        // Constructor
        Inequality(Key key1, const SharedNoiseModel &model) 
        : NoiseModelFactor1<double>(model, key1){}

        // Evaluate the error
        Vector evaluateError(const double &cons_variable,
                             OptionalMatrixType H1) const override
        {
            if (H1)
            {
                *H1 = (Matrix(1, 1) << 1).finished();
            }

            if (cons_variable < 0)
            {
                return (Vector(1) << 1e9).finished();
            }
            return (Vector(1) << 1e-9).finished();
        }
    };
}