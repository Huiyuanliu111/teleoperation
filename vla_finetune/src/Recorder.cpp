/** ------------------------- Revision Code History -------------------
*** Programming Language: C++
*** Description: Data Recorde
*** Released Date: Feb. 2021
*** Hamid Sadeghian
*** h.sadeghian@eng.ui.ac.ir
----------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>

using namespace std;
using namespace Eigen;

class Recorder {
 public:
 Recorder(double t_rec, double sampleTime, int NoDataRec = 10, std::string name = "DATA");
 // Recorder(int NoDataRec = 10, std::string name = "DATA");
  ~Recorder();

  void addToRec(int value);
  void addToRec(double value);
  void addToRec(double array[], int sizeofarray);
  void addToRec(std::array<double, 3> array);
  void addToRec(std::array<double, 6> array);
  void addToRec(std::array<double, 7> array);

  void addToRec(Vector3d& vector);
  void saveData();
  void next();

  void getDAT(Matrix<double, Dynamic, 1>& _Buffer, int rowNum);
  void ensureCapacity(int required_row);
  // void addRow(const Eigen::Ref<const VectorXd>& row);
  // void saveData();
  // int cols() const{return _NoDataRec;}
  // size_t rows() const{return _rows.size();}

  // private:
  // int _NoDataRec;
  // std::string _name;
  // std::vector<VectorXd> _rows;

 private:
  int _index;
  int _columnindex;
  int _rowindex;
  double _t_rec;
  int _NoDataRec;
  std::string _name;
  Matrix<double, Dynamic, Dynamic> _DAT;
};
  // Recorder::Recorder(int NoDataRec, std::string name):_NoDataRec(NoDataRec),_name(std::move(name)){
  //     _rows.reserve(10000);
  // }

  // Recorder::~Recorder(){
  //    saveData();
  // }

  // void Recorder::addRow(const Eigen::Ref<const VectorXd>& row){
  //   if (row.size() != _NoDataRec)
  //   {
  //     throw std::runtime_error("Recorder::addRow -row size mismatch(expected" + std::to_string(_NoDataRec) + ")");
  //   }
  //   _rows.emplace_back(row); 
  // }

  // void Recorder::saveData(){
  //   std::ofstream myfile(_name +".m");
  //   if (!myfile.is_open())
  //   {
  //     throw std::runtime_error("Recorder::savaData - cannnot open file");
  //   }

    
  // myfile << _name <<"m=[\n";
  // for (const auto& r:_rows)
  // {
  //   for (int i = 0; i < r.size(); ++i)
  //   {
  //     myfile << r[i];
  //     if(i +1 < r.size()) myfile << "";
  //   }
  //   myfile << ":\n";
  // }
  // myfile << "];\n";
  // myfile.close();

  // std::cout << "\n\n\t************Data was written successfully ************\n";
  // }


  
                                                    
Recorder::Recorder(double t_rec, double sampleTime, int NoDataRec, std::string name) {
  _DAT.resize((int)(t_rec / sampleTime + 2), NoDataRec);
  _DAT.setZero();
  _rowindex = 0;
  _columnindex = 0;
  _t_rec = t_rec;
  _name = name;
  _NoDataRec = NoDataRec;
};
Recorder::~Recorder() {
  saveData();
};

void Recorder::getDAT(Matrix<double, Dynamic, 1>& _Buffer, int rowNum){
  _Buffer = _DAT.row(rowNum);
}

// **************************************************************
void Recorder::ensureCapacity(int required_row)
{
  if (required_row < _DAT.rows()) return;

  int new_rows = _DAT.rows();
  if (new_rows < 1) new_rows = 1;

  while (new_rows <= required_row)
  {
    new_rows *= 2;
  }

  Matrix<double, Dynamic, Dynamic> newDAT(new_rows, _DAT.cols());
  newDAT.setZero();
  newDAT.topRows(_DAT.rows()) = _DAT;
  _DAT.swap(newDAT);
  
  
}
// **************************************************************

void Recorder::addToRec(int value) {
  ensureCapacity(_rowindex);
  _DAT(_rowindex, _columnindex) = value;
  _columnindex++;
}
void Recorder::addToRec(double value) {
  ensureCapacity(_rowindex);
  _DAT(_rowindex, _columnindex) = value;
  _columnindex++;
}
void Recorder::addToRec(double array[], int sizeofarray) {
  // cout << "TODO: size of array is manual" << endl;
  ensureCapacity(_rowindex);
  for (int i = 0; i < sizeofarray; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
};
void Recorder::addToRec(std::array<double, 7> array) {
  // cout << "TODO: size of array is manual" << endl;
  ensureCapacity(_rowindex);
  for (int i = 0; i < 7; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
};

void Recorder::addToRec(std::array<double, 6> array) {
  // cout << "TODO: size of array is manual" << endl;
  ensureCapacity(_rowindex);
  for (int i = 0; i < 6; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
};
void Recorder::addToRec(std::array<double, 3> array) {
  // cout << "TODO: size of array is manual" << endl;
  ensureCapacity(_rowindex);
  for (int i = 0; i < 3; i++) {
    _DAT(_rowindex, _columnindex) = array[i];
    _columnindex++;
  }
};
void Recorder::addToRec(Vector3d& vector) {
  ensureCapacity(_rowindex);
  for (int i = 0; i < vector.size(); i++) {
    _DAT(_rowindex, _columnindex) = vector[i];
    _columnindex++;
  }
};

void Recorder::saveData() {
  std::ofstream myfile;
  myfile.open(_name + ".m");
  //myfile << _name << "m" <<"=[" << _DAT << "];\n";
  myfile << _name << "m" <<"=[" << _DAT.topRows(_rowindex) << "];\n";
  myfile.close();
  cout << "\n\n\t************Data was written successfully  ************\n";
};
void Recorder::next() {
  _rowindex++;
  _columnindex = 0;
}


