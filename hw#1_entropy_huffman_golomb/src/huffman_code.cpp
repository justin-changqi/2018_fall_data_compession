#include "huffman_code.hpp"

Node::Node(std::string letter, int cnt)
{
  this->letter = letter;
  this->cnt = cnt;
  this->parent = NULL;
  this->child_l = NULL;
  this->child_r = NULL;
}

HuffmanCode::HuffmanCode( std::string file ) 
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
  this->printTable(alphabas, counts);
  this->initNodes(alphabas, counts);
  this->buildTree();
}

void HuffmanCode::printTable(std::vector<char> alphabas, std::vector<int> counts)
{
  std::cout << std::setw(15) << "Alphaba";
  for (int i = 0; i < alphabas.size() / 2; i++)
  {
    std::cout << std::setw(5) << this->getSymbol(alphabas[i]);
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "Total Number";
  for (int i = 0; i < alphabas.size() / 2; i++)
  {
    std::cout << std::setw(5) << counts[i];
  }
  std::cout << std::endl << std::endl;;
  std::cout << std::setw(15) << "Alphaba";
  for (int i = alphabas.size() / 2; i < alphabas.size(); i++)
  {
    std::cout << std::setw(5) << this->getSymbol(alphabas[i]);
  }
  std::cout << std::endl;
  std::cout << std::setw(15) << "Total Number";
  for (int i = alphabas.size() / 2; i < alphabas.size(); i++)
  {
    std::cout << std::setw(5) << counts[i];
  }
  std::cout << std::endl;
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
    Node n(this->getSymbol(alphabas[i]), counts[i]);
    this->nodes.push_back(n);
  }
}

void HuffmanCode::buildTree()
{
  std::sort(this->nodes.begin(), this->nodes.end());
  do
  {
    
  }while(this->nodes.size() <= 0);
  // for (std::vector<Node>::iterator it=this->nodes.begin(); it!=this->nodes.end(); ++it)
  //   std::cout << ' ' << it->letter << ":" << it->cnt;
  // std::cout << '\n';
}

int main(int argc, char const *argv[])
{
  HuffmanCode huffman_code("../santaclaus.txt");
  return 0;
}
