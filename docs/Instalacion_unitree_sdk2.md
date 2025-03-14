# Instalación de SDK2 de Unitree  

## Requerimientos  
- Ubuntu 20.04
- 
## 1️. Clonar el repositorio  
Abre una terminal y ejecuta: 
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
```
## 2. Construir la librería
Ejecuta los siguientes comandos:
```bash
cd unitree_sdk2
mkdir build
cd build
cmake ..
```
## 📌 Resultado esperado
Imagen
## Instalar la librería
```bash
sudo make install
```
## 📌 Resultado esperado
IMAGEN
## Probar la instalación usando Hello World
### Publicador
```bash
cd bin
./test_publisher
```
El terminal debe quedar esperando un suscriptor.

### Suscriptor
Abre otra terminal y ejecuta:
```bash
cd unitree_sdk2/build/bin
./test_subscriber
```
Deben de aparecer mensajes de:
```bash
HelloWorld
```
## Alternativa: Usar sin instalar
Si no quieres instalar la librería para tus propios desarrollos y solo quieres usar sus ejemplos , compila con:

```bash
cd unitree_sdk2
mkdir build
cd build
cmake ..
make
```
Esto te permitirá usar los binarios sin necesidad de instalación.

Para visualizar la referencia original [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2))

## Posibles errores que se pueden presentar
Si aparece el siguiente error:
IMAGEN
Se debe a que no se encuentra asociada la librería instalada correctamente, la cual está
ubicada en **usr/local/lib**. Para visualizar la librería que está asociada se ejecuta:
```bash
ldd ./test_subscriber
```
Si se encuentra una librería con ubicación diferente tal como la siguiente
imagen:

IMAGEN

Se deben ejecutar los siguientes comandos:
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

Una vez solventado el error, se puede ejecutar nuevamente el ejemplo de Hello World.
