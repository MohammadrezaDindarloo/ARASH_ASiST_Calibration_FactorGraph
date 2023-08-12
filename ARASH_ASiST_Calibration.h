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
    class ArashAsistFactor : public NoiseModelFactor3<double, double, double>
    {
        // Private member variables to store ground truth and calibration parameters
        private:
        gtsam::Vector3 groud_truth;
        double phi;
        double psi;
        double d;

        public:
        // Constructor
        ArashAsistFactor(Key key1, Key key2, Key key3, gtsam::Vector3 groud_truth_, double phi_, double psi_, double d_, const SharedNoiseModel &model) 
        : NoiseModelFactor3<double, double, double>(model, key1, key2, key3), groud_truth(groud_truth_), phi(phi_), psi(psi_), d(d_) {}

        // Evaluate the error
        Vector evaluateError(const double &alpha, const double &theta, const double &L,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3) const override
        {
            // Forward kinematic model of robot which contain the variables to optimize (this is type of vector (x, y, z))
            gtsam::Vector3 task_point;
            task_point[0] = 0.1000e4 * L * sin(theta) - 0.1000e4 * d * (cos(alpha + psi) * sin(theta) + sin(alpha + psi) * cos(phi) * cos(theta));
            task_point[1] = -0.1000e4 * d * sin(alpha + psi) * sin(phi);
            task_point[2] = 0.1000e4 * L * cos(theta) - 0.1000e4 * d * (cos(alpha + psi) * cos(theta) - sin(alpha + psi) * cos(phi) * sin(theta));
            gtsam::Vector3 measurement_model(task_point[0], task_point[1], task_point[2]);

            if (H1)
            {   
                // model jacobian with respect to alpha (the first optimization variable)
                gtsam::Vector3 derivative_wrt_alpha;
                derivative_wrt_alpha[0] = -0.1000e4 * d * (-sin(alpha + psi) * sin(theta) + cos(alpha + psi) * cos(phi) * cos(theta));
                derivative_wrt_alpha[1] = -0.1000e4 * d * cos(alpha + psi) * sin(phi);
                derivative_wrt_alpha[2] = -0.1000e4 * d * (-sin(alpha + psi) * cos(theta) - cos(alpha + psi) * cos(phi) * sin(theta));

                *H1 = (gtsam::Matrix(3, 1) << derivative_wrt_alpha[0], derivative_wrt_alpha[1], derivative_wrt_alpha[2]).finished();
            }
            if (H2)
            {
                // model jacobian with respect to theta (the second optimization variable)
                gtsam::Vector3 derivative_wrt_theta;
                derivative_wrt_theta[0] = 0.1000e4 * L * cos(theta) - 0.1000e4 * d * (cos(alpha + psi) * cos(theta) - sin(alpha + psi) * cos(phi) * sin(theta));
                derivative_wrt_theta[1] = 0;
                derivative_wrt_theta[2] = -0.1000e4 * L * sin(theta) - 0.1000e4 * d * (-cos(alpha + psi) * sin(theta) - sin(alpha + psi) * cos(phi) * cos(theta));

                *H2 = (gtsam::Matrix(3, 1) << derivative_wrt_theta[0], derivative_wrt_theta[1], derivative_wrt_theta[2]).finished();
            }
            if (H3)
            {   
                // model jacobian with respect to L (the third optimization variable)
                gtsam::Vector3 derivative_wrt_L;
                derivative_wrt_L[0] = 0.1000e4 * sin(theta);
                derivative_wrt_L[1] = 0;
                derivative_wrt_L[2] = 0.1000e4 * cos(theta);

                *H3 = (gtsam::Matrix(3, 1) << derivative_wrt_L[0], derivative_wrt_L[1], derivative_wrt_L[2]).finished();
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