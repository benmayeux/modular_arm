#define DEBUG_PORT
#ifdef DEBUG_PORT
  #define _CONCAT(x,y) x##y
  #define CONCAT(x,y) _CONCAT(x,y)
  #define DEBUG_FUNC CONCAT(Serial,DEBUG_PORT)
  #define DEBUG_PRINT(x) DEBUG_FUNC.println("DEBUG: " + (String)x)
#else
  #define DEBUG_PRINT(x) x
#endif