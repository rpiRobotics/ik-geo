//WIP


int main(int argc, char **argv)
{

  std::vector<std::pair<std::string, std::vector<double>>> data = read_csv("dat.csv");
  if (data.size() != 57)
  {
    std::cerr << "Invalid input data for IK3P2I. \n";
    return 0;
  }

  double time_avg = 0;

  for (int i = 0; i < (int)data[0].second.size(); i++)
  {
    Kin kin;
    Eigen::Matrix<double, 3, 3> R_06;
    Eigen::Vector3d p_0T;
    std::vector<double> Q;
    Soln soln;
    kin.H << data[0].second[i], data[1].second[i], data[2].second[i], data[3].second[i], data[4].second[i], data[5].second[i],
             data[6].second[i], data[7].second[i], data[8].second[i], data[9].second[i], data[10].second[i], data[11].second[i],
             data[12].second[i], data[13].second[i], data[14].second[i], data[15].second[i], data[16].second[i], data[17].second[i];
    kin.P << data[18].second[i], data[19].second[i], data[20].second[i], data[21].second[i], data[22].second[i], data[23].second[i], data[24].second[i],
             data[25].second[i], data[26].second[i], data[27].second[i], data[28].second[i], data[29].second[i], data[30].second[i], data[31].second[i],
             data[32].second[i], data[33].second[i], data[34].second[i], data[35].second[i], data[36].second[i], data[37].second[i], data[38].second[i];
    R_06 << data[39].second[i], data[40].second[i], data[41].second[i], 
            data[42].second[i], data[43].second[i], data[44].second[i], 
            data[45].second[i], data[46].second[i], data[47].second[i];
    p_0T << data[48].second[i], data[49].second[i], data[50].second[i];
    // Q << data[51].second[i], data[52].second[i], data[53].second[i], data[54].second[i], data[55].second[i], data[56].second[i];
    auto start = std::chrono::steady_clock::now();

    IK_3_parallel_2_intersecting(R_06, p_0T, kin, soln);
    auto end = std::chrono::steady_clock::now();

    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  }

  time_avg /= (int)data[0].second.size();

  std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl;

  return 0;
}


int main(int argc, char** argv) {
  
  double time_avg = 0;

  for (int i = 0; i < 100000; i ++ ) {
    Kin kin;
    Soln soln;
    Eigen::Matrix<double, 3, 1> T;
    Eigen::Matrix<double, 3, 3> R;

    setup(kin, soln, T, R);

    Eigen::Matrix<double, 6, Eigen::Dynamic> Q;
    Eigen::Matrix<double, 5, Eigen::Dynamic> Q_LS;

    auto start = std::chrono::steady_clock::now();

    IK_spherical_2_intersecting(R, T, kin, Q, Q_LS);

    auto end = std::chrono::steady_clock::now();

    time_avg += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  }

  time_avg /= 100000;

  std::cout << "===== \n time (nanoseconds): " << time_avg << std::endl;

  // std::cout << "Q: " << Q << std::endl;
  // std::cout << "Q_LS: " << Q_LS << std::endl;
}


int main(int argc, char * argv[]) {
	return 0;
}