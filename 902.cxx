#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <map>
#include <stdio.h>

int main(int argc, char* argv[])
{
  int n = 0; 
  std::string message;
  while(std::cin >> n >> message) 
  {
    int totalLength = message.length(); 
    std::map<std::string, int> histogram; 

    for (int i = 0; i < totalLength - n + 1; ++i)
    {
      std::string subString = message.substr(i, n); 
      histogram[subString]++; 
    }
    int maxFreq = 0; 
    std::string password; 
    for (std::map<std::string, int>::iterator iterator = histogram.begin(); iterator != histogram.end(); ++iterator)
    {
      if (iterator->second > maxFreq)
      {
        maxFreq = iterator->second; 
        password = iterator->first;
      }
    }
    std::cout << password << std::endl; 
  }
  
  return 0; 
}