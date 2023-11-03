
[//]: # (Comment: Subir esta presentación a https://remarkjs.com/remarkise)

# Destructores (virtual)

* Los destructores hay que declararlos virtuales:
    - Cuando haya relaciones de herencia y se usen punteros a la clase base 
    para apuntar a objetos de la clase derivada
    - Para que se llamen a todos los destructores (derivada y base)
    - Se tiene que declarar virtual en la base,
    - *Pero* es conveniente declararlo virtual en todas las clases derivadas
* Ejemplo en: [Código Fuente](https://bitbucket.org/stapia/ejemplosprogramacionrobotica/src/main/comun/destructores/)

---

# No se debe llamar al destrucctor

* No se debe llamar al destructor explícitamente.
* El siguiente código fuente no es correcto:

```cpp
class OperacionFalla {
    void voy_a_fallar() {
        this->~OperacionFalla();
    }
};
```

* Lo correcto es:

```cpp
class OperacionFalla {
    void voy_a_fallar() {
        throw new std::exception("La razón del fallo");
    }
};
```

Y se hace try/catch en el código cliente.

---

# Métodos virtuales

* *Sobreescriben* la implementación en la base.
* Al declarar uno cualquiera se crea la tabla de métodos virtuales.
* Es una tabla con los punteros a cada uno de los métodos virtuales.
* Cada objeto tiene el puntero a la tabla según su clase.
* Si no se indica `virtual` no se puede hacer `override`.
* Ejemplo en: [Código Fuente](https://bitbucket.org/stapia/ejemplosprogramacionrobotica/src/main/comun/virtuales/)

---

# Interfaces

* CPP: una clase abstracta es una clase que declara un método abstracto (virtual y sin implementación)
* Es decir: `virtual void metodo_abstracto() = 0;`
* No se puede crear objetos que sea de una clase abstracta, pero si de sus derivadas
* CPP: una interfaz es una clase abstracta pura
* Es decir: no tiene atributos y todos sus métodos son abstractos
* Se usa para: polimorfismo y plugins
* Se puede combinar con herencia múltiple (para implementar varias)
* Para descubrir interfaces implementadas se hace `dynamic_cast<i_interfaz*>`
* Ejemplo en: [Código Fuente](https://bitbucket.org/stapia/ejemplosprogramacionrobotica/src/main/comun/interfaces/)

---

# Dependencias e includes (1)

* La separación entre hpp y cpp se hace para:
    - Meter las declaraciones en el hpp y
    - Las implementaciones en el cpp
    - Porque para usar una clase **sólo** hace falta la declaración.
* De esta forma:
    - En el hpp sólo es necesario poner los includes de las *cosas* que aparezcan
    en la declaración de la clase, pero **NO** las que aparezcan en su implementación.
    - Para asegurar esto es *bueno* poner el hpp de la clase como *primer* include.
* PERO:
    - Para *declarar* un **puntero** no es estrictamente necesaria la declaración de su tipo.
    - Se puede hacer una *forward declaration*. Por ejemplo: `class Hola;`
    - La clase `Hola` es un *tipo incompleto*, pero puedo declarar: `Hola *h;`

---

# Dependencias e includes (2)

Al aplicar sistemáticamente *forward declaration* se puede:

* Evitar introducir en una clase dependencias (vía include) de otras
clases cuando éstas se usen como parámetros (retornos) de métodos:

```cpp
#include "Dependencia.hpp"
class UnaClase {
    void mi_metodo(const Dependencia& d);
};
```

```cpp
class Dependencia;
class UnaClase {
    void mi_metodo(const Dependencia* pd);
};
```

---

# Dependencias e includes (3)

Al aplicar sistemáticamente *forward declaration* se puede:

* Ocultar los atributos y otros detalles internos:

```cpp
#include "Dependencia.hpp"
class UnaClase {
    Dependencia d;
};
```

```cpp
class UnaClaseImp;
class UnaClase {
    UnaClaseImp* imp;
};

#include "Dependencia.hpp"
class UnaClaseImp {
    Dependencia d;
};
```

Ejemplo más completo en: [Código Fuente](https://github.com/santiago-tapia/as2_mc/tree/main/as2_mc_timer_tick). Observese que `impl` NO está en includo sino en `src`.

---

# Templates y su instanciación

* Una función, método o clase *template* **NO** existe, ...
* Hasta que se instancia. 

Es decir:

```cpp
template <typename T>
class UnaPlantilla {
    T x;
    void fijar(const T& a) {
        x = a;
    }
};
```

NO existe hasta que:

```cpp
UnaPlantilla<int> obj;
```

Se instancia.

---

# Templates: sólo .hpp

* ¡Cuidado! Los templates de CPP no son los genéricos de Java (ni de guasa...)
* Debido a que las clases templates **NO** existen:
* No resulta práctico separar en hpp y cpp porque hace falta la 
plantilla completa.
* Producen librerías tipo *headers only*.
* Se usan vía `#include` no vía *compilación* (explicar...).

---

# Cosas que se pueden hacer con Templates 

* Las plantillas permiten hacer *polimorfismo*. Por ejemplo:

```cpp
template <typename Iterator>
Iterator::value_type suma(Iterator first, Iterator last) {
    Iterator::value_type resultado = *first;
    ++first;
    for ( ; first != last; ++first ) {
        resultado = resultado + *first;
    }
    return resultado;
}
```

* Es polimorfismos en el sentido de que `value_type` puede ser **cualquier cosa**
que tenga: una operación `=` y otra `+`. 

---

# Sobre templates, concepts y models

* Dentro de un tipo se pueden declarar tipos *locales* (ejemplo anterior)
* Se pueden declarar constantes o atributos de clase (static)
* [...]
* Todos ellos pueden constituir un **concept**. Es decir, el conjunto de propiedades 
(requisitos) para que un tipo de dato pueda ser el parámetro de una plantilla.
* Cuando un tipo de dato cumple un **concept** se dice que: ***"is a model of a concept**.

# Ejemplos prácticos de templates

* Dos ejemplos sobre *subscriptions*. 
* A destacar que: 
    - No hay cpp
    - Se pueden usar enteros como parámetros de template
    - Te puedes *inventar* que es lo que *requieres*, es decir, el *concept*.
* Ejemplo en: [Código Fuente](https://github.com/santiago-tapia/as2_mc/tree/main/as2_mc_input/include/as2_mc_input)

* Ejemplos sobre *publishers* (con instanciación)
* A destacar:
    - Combina un tipo (alias de tipo) con un dato de tipo `string`
    - Combina templates y no templates
    - Sirve para generar *muchas* clases
* Ejemplo en: [Código Fuente](https://github.com/santiago-tapia/as2_mc/tree/main/as2_mc_input/include/as2_mc_output)

---

# Injección de dependencia

* Dependencia vía herencia:
    - Es una dependencia *fuerte* y *poco flexible*
    - Exige la declaración de la clase base (no vale *forward declaration*)
    - Puede ocasionar el problema del *diamante*
* Dependencia vía (composición/agregación)
    - La dependencia se expresa a través de un atributo 
    - El acoplamiento es más reducido
    - Se puede usar *forward declaration*
* DI: Dependency Injection, es una dependencia donde:
    - Existe un método que *injecta* un objeto de la dependencia
    - Opcionalmente se guarda como atributo
    - Se usa para proporcionar la funcionalidad, pero se entiende como 
    algo *externo* por lo que el acoplamiento es bajo
* Ejemplo en: [Código Fuente](https://github.com/santiago-tapia/as2_mc/tree/main/as2_mc_timer_tick)

---

# Síncrono vs Asíncrono 

* Leer: [Socket recv](https://pubs.opengroup.org/onlinepubs/7908799/xns/recv.html)
* Síncrono:
    - Se **bloquea** la ejecución.
    - Bloquear significa que se interrumpe la ejecución del hilo y se queda pendiente de 
    una interrupción a la espera de que se complete alguna operación (por ejemplo, la entrada de
    un mensaje a través de red).
* Asíncrono:
    - La ejecución **no** se bloquea, bien
    - Se retorna inmediatamente un resultado intermedio o parcial, o bien
    - Se registra una callback para cuando se pueda obtener el resultado completo (implica concurrencia).
* Atención:
    - Las llamadas (clientes) a los *services* de ros2 son asíncronas.
    - Y los servidores de *services* de ros2 están en una *callback*... No se pueden interpretar
    como síncronos porque no hay bloqueo.

---

# Middleware

* (Dibujar: push, pull, ...)
* La razón de este diseño están en la concurrencia.
* La sincronización se produce sobre colas de eventos.
* El push sobre los clientes del middleware implica una *callback* (ros2 subcription)
* Mientras que el pull puede ser síncrono o asíncrono sin *callback* (ros2 parameters)

---

# Algunas aclaraciones sobre ROS2

* Suponiendo que tenemos 1 hilo de ejecución:
    - Las *callbacks* son sucesivas, debe acabar una para comenzar la siguiente
    - Eso


---

# Estrategía sobre ROS2 y diseño orientado a objetos

* Cuanto más aislado esté ROS2 mejor (usar DI)
* Cuanto más se parezca a un programa tipo `main` mejor
* Cuanto mejor se identifiquen las entradas/salidas mejor
* Cuanto más modular mejor
