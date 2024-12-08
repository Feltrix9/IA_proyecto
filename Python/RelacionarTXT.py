# Archivos de entrada y salida
nodos_utilizados_file = "Txt/NodosUtilizados.txt"
road_file = "Txt/NY-road-d.txt"
output_file = "Txt/filtered_NY-road-d.txt"




# Extraer nodos únicos de nodosutilizados.txt
nodos_utilizados = set()

with open(nodos_utilizados_file, "r") as file:
    for line in file:
        if line.strip() and "Nodo expandido:" in line:
            # Extraer el ID del nodo expandido
            parts = line.split(":")
            nodo = parts[1].strip().split()[0]
            nodos_utilizados.add(nodo)

# Filtrar conexiones del archivo NY-road-d.txt
with open(road_file, "r") as road, open(output_file, "w") as output:
    output.write(f"264346 730100\n")
    for line in road:
        parts = line.strip().split()
        if len(parts) == 4:  # Validar formato correcto
            nodo_origen, nodo_destino = parts[0], parts[1]
            if nodo_origen in nodos_utilizados or nodo_destino in nodos_utilizados:
                output.write(line)

print(f"Archivo filtrado generado: {output_file}")
