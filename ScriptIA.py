import subprocess
import os
import time  # Importar módulo time para medir el tiempo

# Definir la carpeta que contiene los scripts Python
scripts_folder = "Python"

# Lista de los archivos Python que deseas ejecutar en orden
scripts_to_run = [
    "ListaParadas.py",
    "RelacionarTXT.py",
    "ListaParadasFinal.py"
]

# Función para ejecutar un script Python
def run_script(script_name):
    script_path = os.path.join(scripts_folder, script_name)
    try:
        print(f"Ejecutando {script_name}...")
        # Usar 'python' en lugar de 'python3' para Windows
        result = subprocess.run(['python', script_path], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(f"Salida de {script_name}: {result.stdout}")
    except subprocess.CalledProcessError as e:
        # Manejar el error de salida y decodificar stderr de forma segura
        print(f"Error al ejecutar {script_name}: {e.stderr.encode('utf-8', 'ignore').decode('utf-8')}")
        return False
    return True

# Medir el tiempo de ejecución total
start_time = time.time()  # Registrar el tiempo al inicio

# Ejecutar cada script de la lista
for script in scripts_to_run:
    if not run_script(script):
        print(f"Abortando ejecución debido a error en {script}.")
        break
else:
    print("Todos los scripts se ejecutaron correctamente.")

# Calcular el tiempo total de ejecución
end_time = time.time()  # Registrar el tiempo al final
total_time = end_time - start_time  # Diferencia entre los tiempos

print(f"Tiempo total de ejecución: {total_time:.2f} segundos.")
