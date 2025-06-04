// smoother.hpp
#pragma once

class Smoother {
public:
  Smoother(double initial_value)
    : wn(0.0), zeta(0.0), current_dot(0.0), current(initial_value) {}

  void update(double wn_, double zeta_) {
    wn = wn_;
    zeta = zeta_;
  }

  double compute(double command, double dt) {
    double current_dot2D;
    current_dot2D = wn * wn * (command - current) - 2 * zeta * wn * current_dot;
    // 뒤부터 연산. current_dot2D * dt를 계산한 후 current_dot을 더하여 저장.
    current_dot += current_dot2D * dt;
    current += current_dot * dt;
    return current;
  }

  // private 멤버 변수인 current_dot 값을 외부에서 읽을 수 있도록 미리 getter 함수를 작성해놓음(캡슐화 원칙).
  double getCurrentDot() const {
    return current_dot;
  }


private:
  double wn;
  double zeta;
  double current_dot;
  double current;
}