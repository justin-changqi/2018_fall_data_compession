#include "huffman_code.hpp"

Node::Node(std::string letter, char symbol, int cnt)
{
  this->symbol = symbol;
  this->letter = letter;
  this->cnt = cnt;
  this->child_l = NULL;
  this->child_r = NULL;
}

Node::Node(int cnt, Node *child_l, Node *child_r)
{
  this->cnt = cnt;
  // this->parent = NULL;
  this->child_l = child_l;
  this->child_r = child_r;
}

bool Node::isLeaf()
{
  if (this->child_l == NULL && this->child_r == NULL)
  {
    return true;
  }
  else
  {
    return false;
  }
}

HuffmanCode::HuffmanCode( std::string file, std::string csv_file) 
{
  std::vector<char> alphabas;
  std::vector<int> counts;
  std::ifstream infile;
  char letter[0];
  infile.open (file, std::ios::app);
  while ( infile.peek()  != EOF ) {
    infile.read (letter, 1);
    // counting letter
    std::vector<char>::iterator it;
    it = std::find (alphabas.begin(), alphabas.end(), *letter);
    if (it != alphabas.end())
    {
      size_t index = it - alphabas.begin();
      counts[index] += 1;
    }
    else
    {
      if(letter[0] != 0x0a)
      {
        alphabas.push_back(letter[0]);
        counts.push_back(1);
      }
    }
  }
  infile.close();
  this->initNodes(alphabas, counts);
  // Get reverse list
  std::vector<Node> nodes_r;
  for (int i = this->nodes.size()-1; i >= 0; i--)
  {
    nodes_r.push_back(this->nodes[i]);
  }
  std::vector <double> pmf, cdf;
  this->getPmfCdf(nodes_r, pmf, cdf);
  this->printTable(nodes_r, pmf, cdf);
  std::cout << "\nEntropy: " << this->getEntropy(pmf) << " bits/symbol" << std::endl;
  this->writeToCsv(csv_file, nodes_r, pmf, cdf);
  this->buildTree();
}

void HuffmanCode::getPmfCdf(std::vector<Node> node_list, 
                            std::vector<double> &pmf, 
                            std::vector<double> &cdf)
{
  for (int i = 0; i < node_list.size(); i++)
  {
    pmf.push_back(node_list[i].cnt / this->total_letters);
    if (i == 0)
    {
      cdf.push_back(pmf[0]);
    }
    else
    {
      cdf.push_back(cdf[i-1]+pmf[i]);
    }
  }
}

 double HuffmanCode::getEntropy(std::vector<double> &pmf)
 {
  double entropy = 0;
  for (int i = 0; i < pmf.size(); i++)
  {
    entropy += pmf[i] * log2(pmf[i]);
  }
  return -entropy;
 }

void HuffmanCode::printTable(std::vector<Node> node_list,
                             std::vector<double> &pmf, 
                             std::vector<double> &cdf)
{
  std::cout << std::setw(15) << "Alphaba";
  for (int i = 0; i < node_list.size() / 2; i++)
  {
    std::cout << std::setw(10) << node_list[i].letter;
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "Total Number";
  for (int i = 0; i < node_list.size() / 2; i++)
  {
    std::cout << std::setw(10) << node_list[i].cnt;
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "PMF";
  for (int i = 0; i < node_list.size() / 2; i++)
  {
    std::cout << std::setw(10) << std::setprecision (3)<< pmf[i];
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "CDF";
  for (int i = 0; i < node_list.size() / 2; i++)
  {
    std::cout << std::setw(10) << std::setprecision (3)<< cdf[i];
  }
  std::cout << std::endl << std::endl;;
  std::cout << std::setw(15) << "Alphaba";
  for (int i = node_list.size() / 2; i < node_list.size(); i++)
  {
    std::cout << std::setw(10) << node_list[i].letter;
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "Total Number";
  for (int i = node_list.size() / 2; i < node_list.size(); i++)
  {
    std::cout << std::setw(10) << node_list[i].cnt;
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "PMF";
  for (int i = node_list.size() / 2; i < node_list.size(); i++)
  {
    std::cout << std::setw(10) << std::setprecision (3)<< pmf[i];
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "CDF";
  for (int i = node_list.size() / 2; i < node_list.size(); i++)
  {
    std::cout << std::setw(10) << std::setprecision (3)<< cdf[i];
  }
  std::cout << std::endl;
}

void HuffmanCode::writeToCsv( std::string file_path,
                              std::vector<Node> node_list,
                              std::vector<double> &pmf, 
                              std::vector<double> &cdf)
{
  std::ofstream myfile(file_path);
  myfile << "letter,pmf,cdf" << std::endl;
  for (int i = 0; i < node_list.size(); i++)
  {
    myfile << node_list[i].letter << "," << pmf[i] << "," <<  cdf[i] << std::endl;
  }
  myfile.close();
}

std::string HuffmanCode::getSymbol(char c)
{
  switch(c)
  {
    case 0x0d:
      return "CR";
      break;
    case 0x20: 
      return "SP";
      break;
    default:
      std::string s;
      s += c;
      return s;
      break;
  }
}

void HuffmanCode::initNodes(std::vector<char> alphabas, std::vector<int> counts)
{
  for (int i = 0; i < alphabas.size(); i++)
  {
    Node n(this->getSymbol(alphabas[i]), alphabas[i],counts[i]);
    this->nodes.push_back(n);
  }
  std::sort(this->nodes.begin(), this->nodes.end());
  this->total_letters = 0;
  for (auto& n : this->nodes)
    this->total_letters += n.cnt;
}

void HuffmanCode::buildTree()
{
  int start_index = 0;
  while(this->nodes.size()-1 > start_index)
  {
    int node_merge_cnt = 1;
    Node &node = this->nodes[start_index];
    for (int i = start_index+1; i < this->nodes.size(); i++)
    {
      Node node_next =  this->nodes[i];
      if (node == node_next)
      {
        node_merge_cnt++;
      }
      else
      {
        break;
      }
    }
    if (node_merge_cnt == 1)
    {
      node_merge_cnt = 2;
    }
    else
    {
      node_merge_cnt = (node_merge_cnt / 2) * 2;
    }
    // Merge Node then put into node list
    this->mergeNodesToList(this->nodes, start_index, start_index+node_merge_cnt);
    start_index += node_merge_cnt;
  }
  this->root = &this->nodes.back();
}

void HuffmanCode::mergeNodesToList(std::vector<Node> &list,
                                   int start_indx, int end_indx) 
{
  for (int i = start_indx; i < end_indx; i = i + 2)
  {
    Node *nodes_ptr = this->nodes.data();
    Node *node_i = nodes_ptr+i;
    Node *node_i_1 = nodes_ptr+i+1;
    Node n(list[i]+list[i+1], node_i, node_i_1); 
    if (list.size() > 1)
    {
      bool inserted = false;
      for (int j = 0; j < list.size(); j++)
      {
        if (list[j] >= n)
        {
          list.insert(list.begin()+j, n);
          inserted = true;
          break;
        }
      }
      if (!inserted)
      {
        this->nodes.push_back(n);
      }
    }
    else
    {
      this->nodes.push_back(n);
    }
  }
}

void HuffmanCode::printTree(Node *root, int spaces)
{
  if(root != NULL)
  {
    this->printTree(root->child_r, spaces + 5);
    for(int i = 0; i < spaces; i++)
      std::cout << ' ';
    std::cout << "   " << root->cnt << std::endl;
    this->printTree(root->child_l, spaces + 5);
  }
  else
  {
    return;
  }
}

void HuffmanCode::encodeData(Node *root, std::string code)
{
  if(root != NULL)
  {
    this->encodeData(root->child_r, code+"1");
    this->encodeData(root->child_l, code+"0");
    if (root->isLeaf())
    {
      std::pair <char, std::string> codeword(root->symbol, code);
      this->code_list.push_back(codeword);
    }
  }
  else
  {
    return;
  }
}

void HuffmanCode::printCodeWord()
{
  int cols = 2;
  for (int i = 0; i < code_list.size(); i=i+cols)
  {
    for (int j = i; j < i + cols; j++)
    {
      if (j <  code_list.size())
      {
        std::cout << std::setw(10) << this->getSymbol(this->code_list[j].first) << ": " 
                  << std::setw(10) << this->code_list[j].second;
      }
      else
      {
        break;
      }
    }
    std::cout << std::endl;
  }
}

int main(int argc, char const *argv[])
{
  HuffmanCode huffman_code("../santaclaus.txt", "../statistic_result.csv");
  // HuffmanCode huffman_code("../test.txt",  "../statistic_result.csv");
  // HuffmanCode huffman_code("./hw#1_entropy_huffman_golomb/test.txt",  "../hw#1_entropy_huffman_golomb/statistic_result.csv");
  std::cout << std::endl << "=================== Huffman Tree ================" << std::endl;
  // HuffmanCode huffman_code("./hw#1_entropy_huffman_golomb/test.txt");
  huffman_code.printTree(huffman_code.root, 1);
  std::cout << std::endl << "=================== Codeword Table ================" << std::endl << std::endl;
  huffman_code.encodeData(huffman_code.root, "");
  huffman_code.printCodeWord();
  std::cout << std::endl;
  return 0;
}
