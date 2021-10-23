#pragma once

#include <iostream>

namespace glim {

namespace console {

// foreground
template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& black(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[30m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& red(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[31m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& green(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[32m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& yellow(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[33m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& blue(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[34m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& magenta(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[35m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& cyan(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[36m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& white(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[37m";
  return os;
}

// background
template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bblack(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[40m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bred(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[41m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bgreen(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[42m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& byellow(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[43m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bblue(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[44m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bmagenta(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[45m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bcyan(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[46m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bwhite(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[47m";
  return os;
}

// commands
template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& reset(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[0m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[1m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& underline(std::basic_ostream<CharT, Traits>& os) {
  os << "\033[4m";
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& inverse(std::basic_ostream<CharT, Traits>& os) {
  os << "\034[7m";
  return os;
}

// bold colors

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold_black(std::basic_ostream<CharT, Traits>& os) {
  os << bold << black;
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold_red(std::basic_ostream<CharT, Traits>& os) {
  os << bold << red;
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold_green(std::basic_ostream<CharT, Traits>& os) {
  os << bold << green;
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold_yellow(std::basic_ostream<CharT, Traits>& os) {
  os << bold << yellow;
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold_blue(std::basic_ostream<CharT, Traits>& os) {
  os << bold << blue;
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold_magenta(std::basic_ostream<CharT, Traits>& os) {
  os << bold << magenta;
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold_cyan(std::basic_ostream<CharT, Traits>& os) {
  os << bold << cyan;
  return os;
}

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>& bold_white(std::basic_ostream<CharT, Traits>& os) {
  os << bold << white;
  return os;
}

}  // namespace console
}  // namespace glim