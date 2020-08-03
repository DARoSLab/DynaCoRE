#include "Polyhedron.h"
#include <Optimizer/gurobi/src/gurobi_c++.h>

Polyhedron::Polyhedron()
    : matPtr_(nullptr)
    , polytope_(nullptr)
{
    dd_set_global_constants();
}

Polyhedron::~Polyhedron()
{
    if (matPtr_ != nullptr)
        dd_FreeMatrix(matPtr_);
    if (polytope_ != nullptr)
        dd_FreePolyhedra(polytope_);
    dd_free_global_constants();
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::vrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    isFromGenerators_ = false;
    if (!hvrep(A, b))
        throw std::runtime_error("Bad conversion from hrep to vrep.");

    return vrep();
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::hrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    isFromGenerators_ = true;
    if (!hvrep(A, b))
        throw std::runtime_error("Bad conversion from vrep to hrep.");
    return hrep();
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::vrep()
{
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    return ddfMatrix2EigenMatrix(mat);
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::hrep()
{
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    return ddfMatrix2EigenMatrix(mat);
}

void Polyhedron::printVrep()
{
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    dd_WriteMatrix(stdout, mat);
}

void Polyhedron::printHrep()
{
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    dd_WriteMatrix(stdout, mat);
}

/**
 * Private functions
 */

bool Polyhedron::hvrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    Eigen::MatrixXd cMat = concatenateMatrix(A, b);
    return doubleDescription(cMat);
}

void Polyhedron::initializeMatrixPtr(Eigen::Matrix<int, 1, 1>::Index rows, Eigen::Matrix<int, 1, 1>::Index cols)
{
    if (matPtr_ != nullptr)
        dd_FreeMatrix(matPtr_);
    matPtr_ = dd_CreateMatrix(rows, cols);
    matPtr_->representation = (isFromGenerators_ ? dd_Generator : dd_Inequality);
}

bool Polyhedron::doubleDescription(const Eigen::MatrixXd& matrix)
{
    initializeMatrixPtr(matrix.rows(), matrix.cols());

    for (auto row = 0; row < matrix.rows(); ++row)
        for (auto col = 0; col < matrix.cols(); ++col)
            matPtr_->matrix[row][col][0] = matrix(row, col);

    if (polytope_ != nullptr)
        dd_FreePolyhedra(polytope_);
    polytope_ = dd_DDMatrix2Poly(matPtr_, &err_);
    return (err_ == dd_NoError) ? true : false;
}

Eigen::MatrixXd Polyhedron::concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    double sign = (isFromGenerators_ ? 1 : -1);
    Eigen::MatrixXd mat(A.rows(), A.cols() + 1);
    mat.col(0) = b;
    mat.block(0, 1, A.rows(), A.cols()).noalias() = sign * A;
    return mat;
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::ddfMatrix2EigenMatrix(dd_MatrixPtr mat)
{
    double sign = (isFromGenerators_ ? -1 : 1);
    auto rows = mat->rowsize;
    auto cols = mat->colsize;
    Eigen::MatrixXd mOut(rows, cols - 1);
    Eigen::VectorXd vOut(rows);
    for (auto row = 0; row < rows; ++row) {
        vOut(row) = mat->matrix[row][0][0];
        for (auto col = 1; col < cols; ++col)
            mOut(row, col - 1) = sign * mat->matrix[row][col][0];
    }

    return std::make_pair(mOut, vOut);
}

Eigen::VectorXd Polyhedron::GetChebyshevCenter(Eigen::MatrixXd A, Eigen::VectorXd b) {
    int dimension(A.cols());
    int num_poly(A.rows());
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(dimension+1); //last element is radius
    Eigen::MatrixXd A_cheby = Eigen::MatrixXd::Zero(num_poly, dimension + 1);

    //set A_cheby
    A_cheby.block(0, 0, num_poly, dimension) = A;
    for (int i = 0; i < num_poly; ++i) {
        A_cheby(i, dimension) = sqrt( A(i, 0)*A(i, 0) + A(i, 1)*A(i, 1) );
    }

    //single lp problem
    try{

        GRBEnv env = GRBEnv();
        GRBModel model  = GRBModel(env);
        model.getEnv().set(GRB_IntParam_OutputFlag, 0);
        //GRBVar x[dimension+1];
        GRBVar x[3]; //TODO
        GRBLinExpr cost;

        //set variables
        for (int i = 0; i < dimension; ++i) {
            x[i]=model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
        }
        x[dimension]=model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
        model.update();

        //set cost : minimizing -r
        cost = -1 * x[dimension];
        model.setObjective(cost, GRB_MINIMIZE);

        //set constraint
        for (int i = 0; i < num_poly; ++i) {
            GRBLinExpr w;//need to check
            for (int j = 0; j < dimension+1; ++j) {
                w += A_cheby(i, j) * x[j];
            }
            model.addConstr(w <= b[i]);
        }

        //get solution
        model.optimize();
        for (int i = 0; i < dimension+1; ++i) {
            ret[i] = x[i].get(GRB_DoubleAttr_X);
        }

    }

    catch(GRBException e) {
        std::cout << "Error code in Polyhedron= " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
        exit(0);
    }

    return ret;
}


//cddlib Example
//Polyhedron poly;
//sejong::Matrix AVrep(4,3);
//sejong::Vector bVrep(4);

//AVrep << 1, 1, 2,
//1, -1, 2,
//-1, -1, 2,
//-1, 1, 2;
//bVrep << 0, 0, 0, 0;

//Eigen::MatrixXd AHrep;
//Eigen::VectorXd bHrep;

//auto hrep = poly.hrep(AVrep, bVrep);

//std::cout << hrep.first << std::endl;
//std::cout << hrep.second << std::endl;
