#include <iostream>

class Calculator {
public:
    void Add(float a, float b);
    float GetResult();

protected:
    float result;
};

void Calculator::Add(float a, float b) {
    result = a + b;
}

float Calculator::GetResult() {
    return result;
}

class Calculator2 : public Calculator {
public:
    void Sub(float a, float b);
};

void Calculator2::Sub(float a, float b) {
    result = a - b;
}

int main() {
    Calculator calc;
    calc.Add(4.5, 2.0);
    std::cout << calc.GetResult() << std::endl;

    Calculator2 calc2;
    calc2.Sub(4.2, 4.5);
    std::cout << calc2.GetResult() << std::endl;
    return 0;
}