# Un objeto detectado por el nodo de percepcion (pose en el marco del arreglo padre)
string id              # identificador
string label           # clase asignada segun la variante (color o forma)
string color           # red | green | blue
string shape           # cube | cylinder
float32 area_px        # area del contorno en la imagen [px^2]
geometry_msgs/Pose pose   # centro del objeto, marco 'world' [m]
