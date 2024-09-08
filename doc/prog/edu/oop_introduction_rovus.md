---
marp: true
paginate: true
style: |
  .columns {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 1rem;
  }
---
# <!--fit--> Introduction aux classes (cpp)

---
# <!--fit--> Retour sur les connaissances de base:
<style scoped>section { font-size: 30px; }</style>

## Types principaux: 
```cpp
void, unsigned int, int, float, double, char
```

## Types de bases des "int": 
<div class="columns">
<div>

```cpp
uint8_t  [0, 255]
uint16_t [0, 65535]
uint32_t [0, 4294967295]
```
</div>
<div>

```cpp
int8_t  [-128, 127]
int16_t [-32768, 32767]
int32_t [-2147483648, 214748367]
```
</div>

---
# <!--fit--> Retour sur les connaissances de base:
<style scoped>section { font-size: 28px; }</style>

## Variables
```cpp
int counter = 1;
float angle = 45.92f
int tableau[3] = {0, 1, 2};
```

## Functions
```cpp
int add(int a, int b);

[...]

int add(int a, int b)
{ 
    return a + b; 
}
```

---
# <!--fit--> Retour sur les connaissances de base:
<!-- <style scoped>section { font-size: 28px; }</style> -->

## Le scope
```cpp
int main(void)
{
    char a = 'a';
    {
        char b = 'b';
        {
            char c = 'c';
            printf("%c, %c, %c \n", a, b, c);
        }
        printf("%c, %c, %c \n", a, b, c); // c doesn't exist in this scope
    }
    printf("%c, %c, %c \n", a, b, c); // b and c doesn't exist in this scope
}
```

---
# Définition d'une classe:
<style scoped>section { font-size: 40px; }</style>
Un **type** personnalisé constitué de plusieurs variables et fonctions dans le but de simplifié le code pour l'humain 

---
# Éléments d'une classe:
<style scoped>section { font-size: 40px; }</style>
- Membres (variables)
- Méthodes (functions)

---
# Exemple:
<style scoped>section { font-size: 35px; }</style>

```cpp
class Moteur
{
public:
    Moteur()
    {

    }

    ~Moteur()
    {

    }
};
```
---
# Exemple:
<style scoped>section { font-size: 35px; }</style>

```cpp
class Moteur
{
public:
    Moteur(){}
    ~Moteur(){}
};
```
---
<!-- <style scoped>section { font-size: 30px; }</style> -->
# Exemple:

<div class="columns">
<div>

```cpp
class Moteur
{
public:
    // Membres
    int mode = 1;
    float position = 0.0f;
    float angle = 0.0f;

    // Methodes
    Moteur(){}
    ~Moteur(){}
};
```

</div>
<div>

```cpp
int main(void)
{
    Moteur m1;
    printf("%i \n", m1.mode);
    m1.mode = 25;
    printf("%i \n", m1.mode);
}
```
```cpp
Output: 
    1
    25
```

</div>
</div>

---
<style scoped>section { font-size: 30px; }</style>
# Exemple:
<div class="columns">
<div>

```cpp
class Moteur
{
public:
    // Membres
    float position;
    float lastPosition;
    unsigned long lastTime;

    // Methodes
    Moteur(){}
    ~Moteur(){}
    float getSpeed();
};
```
<!-- float Moteur::getSpeed()
{
    unsigned long dt = getTime() - lastTime;
    float speed = (lastPosition - position) / (float)dt;
    return speed;
} -->

</div>
<div>

```cpp
int main(void)
{
    Moteur m1;
    printf("%f \n", m1.getSpeed());
}
```

```cpp
Output: 10.0
```
</div>
</div>

---
# private:
- Les éléments sont seulements disponibles dans les méthodes de la classe
- Les éléments sont toujours privés si non-spécifié
- Les membres sont généralement tous privés par convention

# protected:
- Les éléments sont accessible seulement dans la class ou par ses enfants*

# public:
- Les éléments sont disponible en dehors de la classe

---
<style scoped>section { font-size: 30px; }</style>
<div class="columns">
<div>

```cpp
class Moteur
{
private:
    // Membres
    float position;
    float lastPosition;
    unsigned long lastTime;

public:
    // Methodes
    Moteur();
    ~Moteur();
    float getSpeed();
};

float Moteur::getSpeed()
{
    unsigned long dt = getTime() - lastTime;
    float speed = (lastPosition - position)/dt;
    return speed;
}
```

</div>
<div>

```cpp
int main(void)
{
    Moteur m1;
    printf("%f \n", m1.position);
}
```

```cpp
Erreur de compilation:
"m1.position is inaccessible"
```
</div>
</div>

---
<style scoped>section { font-size: 30px; }</style>
<div class="columns">
<div>

```cpp
class Moteur
{
private:
    // Membres
    float pos;
    float lastPos;
    unsigned long lastTime;

public:
    // Methodes
    Moteur();
    ~Moteur();
    float getSpeed();
};

float Moteur::getSpeed()
{
    unsigned long dt = getTime() - lastTime;
    float speed = (lastPosition - position)/dt;
    return speed;
}
```

</div>
<div>

```cpp
int main(void)
{
    Moteur m1;
    printf("%f \n", m1.getSpeed());
}
```

```cpp
Output: 10.00
```
</div>
</div>

---
# Opérateur "::" (Namespace)

Ex: 
```cpp
float Moteur::getSpeed()
```

---
# Namespaces
<div class="columns">
<div>

```cpp
namespace LibRovus
{
    int add(int a, int b)
    {
        return a + b;
    }
}

namespace LibInternet
{
    int add(int a, int b)
    {
        printf("arg1: %i, arg2 %i \n", a, b);
        return a + b;
    }
}
```

</div>
<div>

```cpp
int main(void)
{
    printf("%i \n\n", LibRovus::add(1, 2))
    printf("%i \n", LibInternet::add(1, 2))
}
```

<!-- ```cpp
Output: 
    3

    arg1: 1, arg2: 2
    3
``` -->

</div>
</div>

---
# Namespaces
<div class="columns">
<div>

```cpp
namespace LibRovus
{
    int add(int a, int b)
    {
        return a + b;
    }
}

namespace LibInternet
{
    int add(int a, int b)
    {
        printf("arg1: %i, arg2 %i \n", a, b);
        return a + b;
    }
}
```

</div>
<div>

```cpp
int main(void)
{
    printf("%i \n\n", LibRovus::add(1, 2))
    printf("%i \n", LibInternet::add(1, 2))
}
```

```cpp
Output: 
    3

    arg1: 1, arg2: 2
    3
```

</div>
</div>

---
<style scoped>section { font-size: 30px; }</style>
<div class="columns">
<div>

```cpp
class Moteur
{
private:
    // Membres
    float pos;
    float lastPos;
    unsigned long lastTime;

public:
    // Methodes
    Moteur();
    ~Moteur();
    float getSpeed();
};

float Moteur::getSpeed()
{
    unsigned long dt = getTime() - lastTime;
    float speed = (lastPosition - position)/dt;
    return speed;
}
```

</div>
<div>

```cpp
int main(void)
{
    Moteur m1;
    printf("%f \n", m1.getSpeed());
}
```

```cpp
Output: 10.00
```
</div>
</div>

---
<style scoped>section { font-size: 30px; }</style>

# Une classe dans une classe

<div class="columns">
<div>

```cpp
class Vache
{
public:
    Vache() {}
    ~Vache() {}

    void meumeu()
    {
        printf("La vache fait meu");
    }
};

class Ferme
{
public:
    Ferme() {}
    ~Ferme() {}

    Vache monica;
}
```

</div>
<div>

```cpp
int main(void)
{
    Ferme ferme;
    ferme.monica.meumeu();
}
```

```cpp
Output:
    La vache fait meu
```
</div>
</div>
