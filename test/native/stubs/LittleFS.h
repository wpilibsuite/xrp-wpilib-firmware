#pragma once
#include <algorithm>
#include <limits>
#include <map>
#include <string>
inline size_t testWriteLimit = std::numeric_limits<size_t>::max();
inline bool testCorruptClose = false;
inline void (*testBeforeFileWrite)() = nullptr;
class File {
 public:
  explicit File(std::string* contents = nullptr) : contents(contents) {}
  explicit operator bool() const { return contents != nullptr; }
  int available() const { return contents && position < contents->size(); }
  int read() { return static_cast<unsigned char>((*contents)[position++]); }
  size_t print(const char* value) {
    if (testBeforeFileWrite) testBeforeFileWrite();
    writing = true;
    size_t length = std::min(std::string(value).size(), testWriteLimit);
    contents->append(value, length);
    return length;
  }
  size_t print(char value) {
    char text[] = {value, 0};
    return print(text);
  }
  template <typename... T>
  void printf(const char*, T...) {}
  void close() {
    if (writing && testCorruptClose) contents->clear();
    writing = false;
  }

 private:
  std::string* contents;
  size_t position = 0;
  bool writing = false;
};
struct TestLittleFS {
  std::map<std::string, std::string> files;
  bool begin() { return true; }
  bool failOpen = false;
  unsigned writeOpens = 0;
  bool failRename = false;
  File open(const char* path, const char* mode) {
    if (failOpen) return File{};
    if (*mode == 'w') {
      ++writeOpens;
      files[path].clear();
    }
    auto it = files.find(path);
    return File{it == files.end() ? nullptr : &it->second};
  }
  bool remove(const char* path) { return files.erase(path) != 0; }
  bool rename(const char* from, const char* to) {
    if (failRename || !files.count(from)) return false;
    files[to] = files[from];
    files.erase(from);
    return true;
  }
};
inline TestLittleFS LittleFS;
