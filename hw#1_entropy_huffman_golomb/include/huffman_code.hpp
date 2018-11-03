#include <iostream>
#include <fstream>
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <iomanip>
#include <sstream>
#include <string>
#include <numeric>
#include <cstring>

class Node
{
  public:
  std::string letter;
  char symbol;
  int cnt;
  Node *child_r;
  Node *child_l;
  Node(std::string letter, char symbol, int cnt);
  Node(int cnt, Node *child_l, Node *child_r);
  bool isLeaf();
  bool operator<(const  Node & other) 
  {
      return cnt < other.cnt;
  }
  bool operator>=(const  Node & other) 
  {
      return cnt >= other.cnt;
  }
  bool operator==(const  Node & other) 
  {
      return cnt == other.cnt;
  }
  int operator+(const  Node & other)
  {
      return cnt + other.cnt;
  }
};

class HuffmanCode {
  public:
  std::vector<Node> nodes;
  Node *root;
  std::vector< std::pair <char, std::string> > code_list; 
  double total_letters;
  HuffmanCode(std::string file, std::string csv_file);
  void getPmfCdf(std::vector<Node> node_list, 
                 std::vector<double> &pmf, 
                 std::vector<double> &cdf);
  double getEntropy(std::vector<double> &pmf);
  void printTable(std::vector<Node> node_list,
                  std::vector<double> &pmf, 
                  std::vector<double> &cdf);
  void writeToCsv(std::string file_path,
                  std::vector<Node> node_list,
                  std::vector<double> &pmf, 
                  std::vector<double> &cdf);
  std::string getSymbol(char c);
  void initNodes(std::vector<char> alphabas, std::vector<int> counts);
  void buildTree();
  void mergeNodesToList(std::vector<Node> &list, int start_indx, int end_indx);
  void printTree(Node *root, int spaces);
  void encodeData(Node *root, std::string code);
  bool isLeaf(Node *root);
  void printCodeWord();
};
