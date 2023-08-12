#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include </home/mohammad/ARASH_ASiST_Calibration_FactorGraph/ARASH_ASiST_Calibration.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace std;
using namespace gtsam;

int main(int argc, char *argv[])
{

    // Open the CSV file of data and record them in data vector
    // std::ifstream file("/home/mohammad/ARASH_ASiST_Calibration_FactorGraph/data_set/dataset3_for_calibration_fix_d(FK_GT).csv");
    std::ifstream file("/home/mohammad/ARASH_ASiST_Calibration_FactorGraph/data_set/dataset4_for_calibration_fix_d(RCM_GT).csv");
    std::vector<std::vector<double>> data;
    if (file) {
        std::string line;
        while (getline(file, line)) {
            std::stringstream ss(line);
            std::vector<double> row;
            std::string val;
            while (getline(ss, val, ',')) {
                row.push_back(stod(val));
            }
            data.push_back(row);
        }
    std::cout << "Number of data: " << data.size() << std::endl;
    } else {
        std::cout << "Unable to open file." << std::endl;
    }

    // create nonlinear factor graph
    NonlinearFactorGraph graph;
    Values initial_estimate;
    
    // test data 
    // std::cout << data[1][0] << std::endl; 
    // std::cout << data[1][1] << std::endl;    
    // std::cout << data[1][2] << std::endl;    
    // std::cout << data[1][3] << std::endl;    
    // std::cout << data[1][4] << std::endl;    
    // std::cout << data[1][5] << std::endl;    
   
    // Noise model for our measurement. here we have two type of noise model. 
    // This is the first NoiseModel for sensor measurement
    noiseModel::Gaussian::shared_ptr arash_asist_noise_model = noiseModel::Isotropic::Sigma(3, 0.001);
    // This is the second NoiseModel for InequaliyFactor
    noiseModel::Gaussian::shared_ptr inequality_noise_model = noiseModel::Isotropic::Sigma(1, 0.0001);

    // add factors to our graph
    for (size_t i = 0; i < data.size()-1; i++)
    {
        double d = data[i][0];
        double phi = data[i][1];
        double psi = data[i][2];
        gtsam::Vector3 groud_truth(data[i][3], data[i][4], data[i][5]);

        graph.add(std::make_shared<ArashAsistFactor>(Symbol('x', 0), Symbol('x', 1), Symbol('x', 2), groud_truth, phi, psi, d, arash_asist_noise_model));
    }

    // add cosntrain factor in necessary
    // graph.add(std::make_shared<Inequality>(Symbol('x', 0), inequality_noise_model));
    // graph.add(std::make_shared<Inequality>(Symbol('x', 1), inequality_noise_model));
    // graph.add(std::make_shared<Inequality>(Symbol('x', 2), inequality_noise_model));

    // add initial values for optimization variables
    initial_estimate.insert(Symbol('x', 0), 50 * M_PI/180.0); // alpha
    initial_estimate.insert(Symbol('x', 1), 40.0 * M_PI/180.0); // theta
    initial_estimate.insert(Symbol('x', 2), 350.0); // L

    /*In the following we have three ways for solving our optimization graph. Just use one of them.*/
    // Optimize using Levenberg-Marquardt optimization
    LevenbergMarquardtOptimizer optimizer(graph, initial_estimate);
    Values result_LM = optimizer.optimize();
    std::cout << "using Levenberg-Marquardt optimization:\n";
    result_LM.print();
    double totalError = optimizer.error();
    std::cout << "\nErrorTotal" << totalError << "\nErrorTotalPerGraph" << totalError/optimizer.graph().size() <<std::endl;
    std::cout << "\nAlpha is: " << (result_LM.at<double>(Symbol('x', 0)) ) * 180/M_PI << std::endl;
    std::cout << "\nTheta is: " << (result_LM.at<double>(Symbol('x', 1)) ) * 180/M_PI << std::endl;
    std::cout << "\nL is: " << (result_LM.at<double>(Symbol('x', 2)) ) * 1000 << std::endl;

    // Optimize using Gauss-Newton optimization
    // gtsam::GaussNewtonOptimizer optimizer(graph, initial_estimate);
    // Values result_GN = optimizer.optimize();
    // std::cout << "using Gauss-Newton optimization:\n";
    // double totalError = optimizer.error();
    // std::cout << "\nErrorTotal" << totalError << "\nErrorTotalPerGraph" << totalError/optimizer.graph().size() <<std::endl;
    // // result_GN.print();
    // std::cout << "\nAlpha is: " << (result_GN.at<double>(Symbol('x', 0)) ) * 180/M_PI << std::endl;
    // std::cout << "\nTheta is: " << (result_GN.at<double>(Symbol('x', 1)) ) * 180/M_PI << std::endl;
    // std::cout << "\nL is: " << (result_GN.at<double>(Symbol('x', 2)) ) * 1000 << std::endl;


    // Optimize using isam
    // ISAM2Params isam_params;
    // isam_params.factorization = ISAM2Params::CHOLESKY;
    // isam_params.relinearizeSkip = 1;
    // ISAM2 isam(isam_params);
    // isam.update(graph, initial_estimate);
    // Values result_isam = isam.calculateEstimate();
    // std::cout << "using isam optimization:\n";
    // result_isam.print();
    // std::cout << "\nAlpha is: " << (result_isam.at<double>(Symbol('x', 0)) ) * 180/M_PI << std::endl;
    // std::cout << "\nTheta is: " << (result_isam.at<double>(Symbol('x', 1)) ) * 180/M_PI << std::endl;
    // std::cout << "\nL is: " << (result_isam.at<double>(Symbol('x', 2)) ) * 1000 << std::endl;

    return 0;
}
