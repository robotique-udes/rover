# Table of content

- [Table of content](#table-of-content)
- [CPP Coding guidelines](#cpp-coding-guidelines)
  - [Do not use dynamic allocation on microcontrollers](#do-not-use-dynamic-allocation-on-microcontrollers)
    - [Do:](#do)
    - [Avoid:](#avoid)
  - [CPP Name Style](#cpp-name-style)
  - [Always use full namespace](#always-use-full-namespace)
    - [Do:](#do-1)
    - [Avoid:](#avoid-1)
  - [Use the this-\> pointer when calling a method inside a class](#use-the-this--pointer-when-calling-a-method-inside-a-class)
    - [do:](#do-2)
    - [Avoid:](#avoid-2)
  - [Always specified the void argument in function with "no arguments"](#always-specified-the-void-argument-in-function-with-no-arguments)
    - [do:](#do-3)
    - [avoid:](#avoid-3)
  - [Use .hpp and .cpp file extension if using cpp feature](#use-hpp-and-cpp-file-extension-if-using-cpp-feature)
  - [Enum should always be enum class](#enum-should-always-be-enum-class)
    - [do:](#do-4)
    - [avoid:](#avoid-4)
  - [Enum class should only be casted into their underlying int type](#enum-class-should-only-be-casted-into-their-underlying-int-type)
    - [do:](#do-5)
    - [avoid:](#avoid-5)
  - [Macros](#macros)
  - [Instead of using the const keyword, either use define or constexpr](#instead-of-using-the-const-keyword-either-use-define-or-constexpr)
    - [do:](#do-6)
    - [avoid:](#avoid-6)
  - [Namespaces](#namespaces)

# CPP Coding guidelines

Guidelines are there to help organise and increase readability and maintainability. The following are guidelines, not strict rules, you should always try to conform to them but they can always be broken if it increase readability or maintainability.

## Do not use dynamic allocation on microcontrollers
Using dynamic allocation can fragment the heap memory ([more info](https://stackoverflow.com/questions/3770457/what-is-memory-fragmentation)), it's generally always a bad idea on system which requires a high level of reliability for long period of time. For this reason, the use of the new, delete, std::share_ptr, std::unique_ptr, etc. is discouraged on the rover. The std library use a lot of dynamic allocation so it's generally a bad idea to use it.

### Do:
```cpp
int8_t myInt = 8;
int8_t* myIntPtr = &myInt;

something(&myInt);
```

```cpp
std::array<int, 3> myArray(1, 2, 3); // std::array is static
printf("%i", myArray[2]);
```

### Avoid:
```cpp
int8_t* myInt = new int; // Using dynamic allocation
*myInt = 6;

something(myInt);
```

```cpp
std::vector<int> myVector; // Vector is dynamically allocated
myVector.append(1);
myVector.append(2);
myVector.append(3);
```

On ROS code it's tolerated when it make the code easier to maintain and to read as ROS is full of dynamic allocation and the rover computer has 16gb of RAM.

## CPP Name Style

| Type of data            | Styling               | Example                |
|-------------------------|-----------------------|------------------------|
| variable                | camelCase             | ```myInt```            |
| global variable         | g_camelCase           | ```g_myInt```          |
| function name           | camelCase             | ```myFunc()```         |
| class and namespaces    | PascalCase            | ```MyClass```          |
| private member variable | _camelCase            | ```_myPrivateMember``` |
| public member variable  | camelCase             | ```myPublicMember```   |
| Macro                   | UPPER_CASE_SNAKE_CASE | ```MY_MACRO()```       |
| Constant and define     | UPPER_CASE_SNAKE_CASE | ```PI```               |
| Struct                  | sPascalCase           | ```sMyStruct```        |
| Enum                    | ePascalCase           | ```eMyEnum```          |
| Enum elements           | UPPER_CASE_SNAKE_CASE | ```FIRST_ELEM```       |

## Always use full namespace

Using full namespace increase readability for an external developer.

### Do:

```cpp
#include "rover_can_lib/rover_can_lib.hpp"
[...]
    if (canBus.isOk() && msgPropCmd.data.enable)
    {
        if (DEVICE_ID == (uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR)
        {
            speedCmd = cmdAverage.addValue(msgPropCmd.data.targetSpeed * -100.0f);
        }
    }
[...]
```

### Avoid:

```cpp
#include "rover_can_lib/rover_can_lib.hpp"

using RoverCanLib::Constant::eDeviceId;

[...]
    if (canBus.isOk() && msgPropCmd.data.enable)
    {
        if (DEVICE_ID == (uint16_t)REARRIGHT_MOTOR)
        {
            speedCmd = cmdAverage.addValue(msgPropCmd.data.targetSpeed * -100.0f);
        }
    }
[...]
```

## Use the this-> pointer when calling a method inside a class

### do:
```cpp
// inside a class
calculateSpeed(this->getPosition());
```

### Avoid:
```cpp
// inside a class
calculateSpeed(getPosition()); // It's ambiguous if getPosition is a method from the class or a global function
```

## Always specified the void argument in function with "no arguments"
[Learn why it's better](https://www.reddit.com/r/C_Programming/comments/6ylpct/should_i_use_the_void_keyword_in_functions_that/)
### do:
```cpp
void doNothing(void)
{
    return;
}
```

### avoid:
```cpp
void doNothing()
{
    return;
}
```

## Use .hpp and .cpp file extension if using cpp feature
Example for a class named MyClass
```cpp
my_class.hpp
my_class.cpp
```

## Enum should always be enum class

Enum class allows users to scope enums and force the use of enum members with their namespace

### do:
```cpp
enum class eDataIndex : uint8_t
{
    MSG_ID = 0x00,
    MSG_CONTENT_ID = 0x01,
    START_OF_DATA = 0x02
};

uint8_t msg[MSG_LENGTH] = {0};
msg[(uint8_t)eDataIndex::MSG_CONTENT_ID] = someDeviceId;
```

### avoid:
```cpp
enum eDataIndex : uint8_t
{
    MSG_ID = 0x00,
    MSG_CONTENT_ID = 0x01,
    START_OF_DATA = 0x02
};

uint8_t msg[MSG_LENGTH] = {0};
msg[MSG_CONTENT_ID] = someDeviceId; // Where MSG_CONTENT_ID is ambiguous 
```

## Enum class should only be casted into their underlying int type

### do:
```cpp
enum class eDataIndex : uint8_t
{
    MSG_ID = 0x00,
    MSG_CONTENT_ID = 0x01,
    START_OF_DATA = 0x02
};

uint8_t msg[MSG_LENGTH] = {0};
msg[(uint8_t)eDataIndex::MSG_CONTENT_ID] = someDeviceId;
```

### avoid:
```cpp
enum class eDataIndex : uint8_t
{
    MSG_ID = 0x00,
    MSG_CONTENT_ID = 0x01,
    START_OF_DATA = 0x02
};

uint8_t msg[MSG_LENGTH] = {0};
msg[(uint16_t)eDataIndex::MSG_CONTENT_ID] = someDeviceId;
```

## Macros

## Instead of using the const keyword, either use define or constexpr

[great explanation here](https://stackoverflow.com/questions/13346879/const-vs-constexpr-on-variables).
Constexpr are suggested over #define because they can be scoped inside class or namespace which make their use less ambiguous. They also have a type which make type mistakes less likely then define. Both are still tolerated

### do:
```cpp
#define PHIL // good example where a define is useful

namespace KEYBINDING
{
#if defined (PHIL) // define are great for preprocessor directives
    constexpr uint8_t JOINT_SELECT_INC = rover_msgs::msg::Joy::CROSS_UP; // can be access only by it's namespace: KEYBINDING::JOINT_SELECT_INC
    constexpr uint8_t JOINT_SELECT_DEC = rover_msgs::msg::Joy::CROSS_DOWN;
[...]
}
```
### avoid:
```cpp
#define JOINT_SELECT_DEC rover_msgs::msg::Joy::CROSS_DOWN; // Tolerated
const uint8_t JOINT_SELECT_INC = rover_msgs::msg::Joy::CROSS_UP; // Don't
```    

## Namespaces
