import subprocess



# Abrir el archivo de salida en modo de escritura (creará el archivo si no existe)
with open('Txt/output2.txt', 'w') as archivo_salida:
    # Leer el archivo línea por línea
    with open('Txt/NY-queries-2p.txt', 'r') as archivo:
        for linea in archivo:
            # Eliminar saltos de línea y espacios extra
            linea = linea.strip()
            # Dividir la línea en una lista de números
            stops = linea.split()
            # Ejecutar el programa C y redirigir la salida al archivo
            result = subprocess.run(['C/RetoIA - Final.exe'] + stops, stdout=archivo_salida, stderr=archivo_salida)
            
