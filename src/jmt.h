vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    Eigen::MatrixXd TM(3,3);
    Eigen::VectorXd coef(3);
    Eigen::VectorXd sf_target(3);
    double t2 = T * T;
    double t3 = T * T * T;
    double t4 = t3 * T;
    double t5 = t4 * T;

    TM <<   t3,   t4,      t5,
            3*t2, 4*t3,    5*t4,
            6*T,  12 * t2, 20 *t3;
    Eigen::MatrixXd TM_ = TM.inverse();

    sf_target << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * t2),
                 end[1] - (start[1] + start[2] * T),
                 end[2] - start[2];

    coef = TM_ * sf_target;

    return {start[0],start[1], 0.5 * start[2], coef[0],coef[1],coef[2]};

}

struct JMT_CFG {

		vector<double> start;
		vector<double> end;
		double T;
};
