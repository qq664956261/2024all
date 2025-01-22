#ifndef _LOG4CXX_ENCRYPTION_H
#define _LOG4CXX_ENCRYPTION_H


#include <iostream>
#include <string>
#include <vector>
namespace log4cxx
{

  class  Encryption 
  {
    private:
      bool encryption;
      std::vector<unsigned char> key; // AES-256
      std::vector<unsigned char> iv;
    public:
      Encryption();
      void handleErrors();
      std::vector<unsigned char> encrypt(const std::string& plaintext, const std::vector<unsigned char>& key, std::vector<unsigned char>& iv);
      std::string decrypt(const std::vector<unsigned char>& ciphertext, const std::vector<unsigned char>& key, const std::vector<unsigned char>& iv);
      bool& getEncryption(){return encryption;};
      std::vector<unsigned char>& getKey(){return key;};
      std::vector<unsigned char>& getIv(){return iv;};
      
  };

}  //namespace log4cxx

#endif //_LOG4CXX_ENCRYPTION_H
