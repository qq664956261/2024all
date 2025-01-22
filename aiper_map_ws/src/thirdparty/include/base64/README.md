# base64 (C++)

Base64 encoding and decoding with c++

## See also

https://renenyffenegger.ch/notes/development/Base64/Encoding-and-decoding-base-64-with-cpp


encode
std::string base64_encode(std::string const& s, bool url = false);
decode
std::string base64_decode(std::string const& s, bool remove_linebreaks = false);

