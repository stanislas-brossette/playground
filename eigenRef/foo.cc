#include <Eigen/Core>
#include <iostream>

void f(Eigen::Block<Eigen::MatrixXd>& M) { M << 1, 2, 3; }

int main(void)
{
    Eigen::MatrixXd A(5, 5);
    Eigen::MatrixXd placeHolder(1, 3);
    placeHolder.setOnes();
    A.setRandom();
    std::cout << "A: " << A << std::endl;
    std::cout << "placeHolder: " << placeHolder << std::endl;
    Eigen::Block<Eigen::MatrixXd> aBlock = placeHolder.block(0, 0, 1, 3);
    if (1) aBlock = Eigen::Block<Eigen::MatrixXd>(A.block(2, 2, 1, 3));
    std::cout << "placeHolder: " << placeHolder << std::endl;
    f(aBlock);
    std::cout << "A: " << A << std::endl;
    std::cout << "placeHolder: " << placeHolder << std::endl;

    return 0;
}
