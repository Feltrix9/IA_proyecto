import subprocess



# Abrir el archivo de salida en modo de escritura (creará el archivo si no existe)
with open('Txt/output.txt', 'w') as archivo_salida:
    # Leer el archivo línea por línea
    with open('Txt/NY-queries-2p.txt', 'r') as archivo:
        #instancia_id = 1
        for linea in archivo:
            # Eliminar saltos de línea y espacios extra
            linea = linea.strip()
            # Dividir la línea en una lista de números
            stops = linea.split()

            #archivo_salida.write(f'Instancia {instancia_id}\n')
            # Ejecutar el programa C y redirigir la salida al archivo
            result = subprocess.run(['C/RetoIA.exe'] + stops, stdout=archivo_salida, stderr=archivo_salida)

            #instancia_id += 1
            
