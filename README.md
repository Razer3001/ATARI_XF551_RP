# ATARI_XF551_RP

Bridge Atari SIO basado en **RP2040 + ESP32 + unidad física XF551/1050**, orientado a mejorar compatibilidad, estabilidad y rendimiento en lectura, escritura y formateo.

Este proyecto busca que un Atari 8-bit pueda trabajar con una disquetera física a través de una arquitectura distribuida:

- **RP2040**: interfaz principal con el bus **SIO** del Atari.
- **ESP32 Master**: coordinación de comandos, timings y enlace con el Slave.
- **ESP32 Slave**: acceso a la unidad física **XF551/1050**, lectura/escritura/formato y gestión de densidad.

---

## Objetivo del proyecto

El objetivo es construir un bridge confiable entre el Atari y una unidad física, manteniendo el comportamiento esperado por distintos DOS del ecosistema Atari, especialmente en operaciones críticas como:

- boot del sistema
- lectura de sectores
- escritura de sectores
- formato de disco
- copia de archivos
- copia de discos
- cambio de disco sin reinicio completo del sistema

---

## Estado actual

El proyecto ha pasado por varias iteraciones para mejorar compatibilidad y velocidad.

Actualmente, la línea de trabajo más estable se basa en:

- **RP_01** como base confiable para:
  - `WRITE`
  - `FORMAT`
  - `READ/WRITE PERCOM`
- mejoras posteriores para:
  - limpieza de `STATUS` después de operaciones exitosas
  - manejo de estado transitorio
  - mejora de timings
  - caché de lectura y optimizaciones de boot

### Compatibilidad observada durante pruebas

Se han realizado pruebas con distintos DOS y escenarios reales de uso, incluyendo:

- **DOS 2.5**
- **MyDOS 4.55**
- **SuperDOS 5.1**

En distintas ramas y versiones se ha logrado:

- boot correcto
- carga de aplicaciones y juegos
- formateo funcional
- escritura funcional

Los principales desafíos encontrados han sido:

- error **163** después de un format correcto
- comportamiento errático al cambiar de disco o de SO
- confirmación incorrecta de densidad DD/ED
- degradación de velocidad por timings demasiado conservadores

---

## Arquitectura general

### 1. RP2040 Master SIO

Responsable de:

- escuchar el bus SIO del Atari
- interpretar comandos (`READ`, `WRITE`, `STATUS`, `PERCOM`, `FORMAT`)
- enviar comandos al ESP32 Master
- responder al Atari con `ACK`, `COMPLETE`, `ERROR` y datos
- aplicar lógica de geometría visible al host

### 2. ESP32 Master Bridge

Responsable de:

- recibir comandos del RP2040
- reenviarlos al Slave
- administrar timings del enlace interno
- coordinar frames, chunks y respuestas
- mantener el canal rápido y estable

### 3. ESP32 Slave Bridge

Responsable de:

- interactuar con la unidad física XF551/1050
- ejecutar `READ`, `WRITE`, `FORMAT`, `STATUS`, `PERCOM`
- detectar y normalizar densidad
- gestionar cachés de lectura
- resolver confirmación real de densidad
- preparar estados limpios visibles al Atari después de operaciones exitosas

---

## Estructura del proyecto

Una estructura típica del repositorio es:

```text
ATARI_XF551_RP/
├── RP2040_Master_SIO/
│   ├── RP2040_Master_SIO.ino
│   └── BridgeProtocol.h
├── ESP32_Master_Bridge/
│   ├── ESP32_Master_Bridge.ino
│   └── BridgeProtocol.h
├── ESP32_Slave_Bridge/
│   ├── ESP32_Slave_Bridge.ino
│   └── BridgeProtocol.h
└── README.md
```

---

## Funcionalidades principales

### Lectura de sectores

- lectura remota desde la unidad física
- soporte de lectura por sectores
- caché de boot para sectores iniciales
- caché por pista para mejorar acceso secuencial
- soporte de lectura repetida con menor latencia

### Escritura de sectores

- escritura sector a sector
- confirmación remota por comando
- verificación en sectores críticos según la rama utilizada
- manejo de errores con reintentos conservadores
- posibilidad de usar caché de sector recién escrito para lecturas inmediatas

### Formato

- soporte de formato en densidades usadas por el proyecto
- compatibilidad con flujos reales de DOS Atari
- generación y devolución de payload final de format
- limpieza de `STATUS` posterior al format para evitar falsos errores visibles al Atari

### PERCOM

- lectura y escritura de bloques PERCOM
- actualización de geometría visible al host
- saneamiento de ciertos casos problemáticos en algunas ramas experimentales

### Estado lógico limpio (`STATUS`)

Para evitar errores falsos después de operaciones correctas, algunas versiones incorporan:

- `STATUS` limpio temporal después de `FORMAT OK`
- `STATUS` limpio temporal después de `WRITE OK`
- normalización de densidad visible (`SD`, `ED`, `DD`)

---

## Problemas abordados durante el desarrollo

### Error 163 después de format correcto

En varias pruebas el disco quedaba físicamente bien formateado, pero el Atari terminaba mostrando error **163**.

La causa principal observada fue el **cierre lógico visible al DOS**, no el formato físico en sí.

Se trabajó en:

- payload final del format
- `STATUS` limpio post-format
- `STATUS` limpio post-write
- reducción de estados sucios arrastrados entre operaciones

### Cambio de disco / cambio de SO

Se observó comportamiento errático al:

- cambiar disquete
- cambiar de DOS
- volver a leer o escribir sin reiniciar el Slave

Para mitigar esto se introdujeron versiones con:

- limpieza de estado transitorio
- reset suave de cachés
- detección de ventana de cambio de medio

### DD candidato vs DD confirmado

En ramas experimentales apareció un problema donde el sistema anunciaba DD demasiado pronto:

- `PERCOM` sugería DD
- `STATUS` sugería DD
- pero la unidad real aún respondía en 128 bytes

Se trabajó en separar:

- **DD candidato**
- **DD confirmado por lectura real**

---

## Enfoque recomendado de desarrollo

Con base en las pruebas realizadas, el enfoque recomendado es:

- mantener **RP_01** como base estable para:
  - `WRITE`
  - `FORMAT`
  - `PERCOM`
- portar solo mejoras puntuales desde ramas experimentales para:
  - `READ`
  - `BOOT`
  - caché
  - `STATUS` limpio
- evitar mezclar sin control ramas que cambian simultáneamente:
  - formato
  - escritura
  - densidad
  - timings

---

## Requisitos de hardware

- Atari 8-bit con bus SIO
- RP2040
- ESP32 Master
- ESP32 Slave
- unidad física Atari **XF551** o **1050**
- cableado SIO / UART / alimentación según el montaje usado

---

## Compilación

### Entorno sugerido

- **Arduino IDE** o plataforma compatible
- soporte para placas:
  - RP2040
  - ESP32

### Archivos a compilar

1. `RP2040_Master_SIO/RP2040_Master_SIO.ino`
2. `ESP32_Master_Bridge/ESP32_Master_Bridge.ino`
3. `ESP32_Slave_Bridge/ESP32_Slave_Bridge.ino`

### Recomendaciones

- usar la misma versión de `BridgeProtocol.h` en los tres módulos
- validar baudrates y timings después de cada cambio importante
- probar primero con lectura y boot antes de tocar write/format

---

## Flujo recomendado de pruebas

### 1. Lectura / boot

- arrancar con disco de sistema
- validar lectura de sectores 1, 2, 3
- validar lectura repetida de sectores altos (por ejemplo 359, 360, 361)

### 2. Formato

- formatear disco nuevo
- verificar que el Atari no quede en error
- comprobar el disco desde DOS

### 3. Escritura

- copiar archivos pequeños
- copiar archivos grandes
- validar sectores de sistema después de la operación

### 4. Cambio de disco

- retirar disco
- insertar otro disco
- repetir lectura, boot o format sin reiniciar el Slave

---

## Depuración

El proyecto se apoya fuertemente en logs serie.

### Logs típicos del RP2040

- comandos SIO detectados
- geometría antes de leer o escribir
- ACK desde Master
- chunks recibidos
- errores de longitud o timeout

### Logs típicos del Slave

- densidad detectada
- PERCOM leído/aplicado
- format iniciado/completado
- verify OK / write OK
- status limpio armado/enviado

---

## Roadmap sugerido

- [ ] detectar cambio de disco de forma más robusta
- [ ] mejorar velocidad general de lectura
- [ ] mejorar velocidad de escritura sin afectar estabilidad
- [ ] reducir necesidad de reiniciar el Slave en escenarios erráticos
- [ ] afinar caché por pista y prefetch
- [ ] consolidar una sola rama estable para SD / ED / DD
- [ ] documentar cableado y configuración final de hardware

---

## Notas

Este proyecto ha evolucionado a partir de pruebas reales sobre hardware, por lo que algunas ramas contienen correcciones muy específicas para compatibilidad con ciertos DOS, densidades y patrones de acceso.

La recomendación es mantener una rama estable y aplicar mejoras de manera incremental, validando siempre:

- boot
- format
- write
- copy
- cambio de disco

---

## Licencia

Definir según preferencia del autor.

Ejemplos:

- MIT

---

## Autor

Proyecto desarrollado y afinado a partir de pruebas reales de integración Atari XF551/1050 con RP2040 + ESP32.
Eduardo Quintana Iturriaga
Chile
