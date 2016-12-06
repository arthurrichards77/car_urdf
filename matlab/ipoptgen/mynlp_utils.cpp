// Copyright (C) 2005, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: hs071_nlp.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-16

#include "mynlp.hpp"

#include <cassert>
#include <iostream>
#include <fstream>

using namespace Ipopt;

HS071_NLP::HS071_NLP()
{

    std::cout << "Hello" << std::endl;
    
#include "mynlp_params_init.cpp"

}

HS071_NLP::~HS071_NLP()
{}

void HS071_NLP::finalize_solution(SolverReturn status,
                                  Index n, const Number* x, const Number* z_L, const Number* z_U,
                                  Index m, const Number* g, const Number* lambda,
                                  Number obj_value,
				  const IpoptData* ip_data,
				  IpoptCalculatedQuantities* ip_cq)
{
  // here is where we would store the solution to variables, or write to a file, etc
  // so we could use the solution.

  std::ofstream myfile;
  myfile.open ("XOPT.dat");
  for (Index i=0; i<n; i++) {
    myfile << x[i] << "  ";
  }
  myfile.close();
  
  // For this example, we write the solution to the console
  std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
  for (Index i=0; i<n; i++) {
     std::cout << "x[" << i << "] = " << x[i] << std::endl;
  }

  //std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
  //for (Index i=0; i<n; i++) {
  //  std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
  //}
  //for (Index i=0; i<n; i++) {
  //  std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
  //}

  std::cout << std::endl << std::endl << "Objective value" << std::endl;
  std::cout << "f(x*) = " << obj_value << std::endl;

  //std::cout << std::endl << "Final value of the constraints:" << std::endl;
  //for (Index i=0; i<m ;i++) {
  //  std::cout << "g(" << i << ") = " << g[i] << std::endl;
  //}
}
