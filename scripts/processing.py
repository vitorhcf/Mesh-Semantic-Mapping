import open3d as o3d
import numpy as np

# Configurações
nome_do_ficheiro = "table.ply" 
nome_saida_stl = "table_mesh.stl"
alpha = 0.06  # Parâmetro da Mesh: menor = mais detalhe/buracos, maior = mais sólido/bolha
simplificacao_triangulos = 1500 # Quantos polígonos queremos no final

pcd = o3d.io.read_point_cloud(nome_do_ficheiro)

if pcd.is_empty():
    print(f"ERRO: Não consegui ler '{nome_do_ficheiro}'.")
    exit()

# =========================================================================
# FASE 1 & 2: FILTRAGEM (Mantida igual à tua)
# =========================================================================
print(f"-> Nuvem original: {len(pcd.points)} pontos.")

# Limpeza de ruído
pcd_limpo, _ = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=0.5)
pcd_limpo = pcd_limpo.voxel_down_sample(voxel_size=0.02)

# =========================================================================
# SEPARAÇÃO (DBSCAN) E GERAÇÃO DE MESH
# =========================================================================
print("\nA agrupar instâncias e a gerar Mesh (STL)...")
labels = np.array(pcd_limpo.cluster_dbscan(eps=0.12, min_points=20, print_progress=False))

# Criamos uma mesh vazia para juntar todos os pedaços (mesa + cadeiras se houver)
mesh_final = o3d.geometry.TriangleMesh()

if len(labels) > 0 and labels.max() != -1:
    max_label = labels.max()
    
    for i in range(max_label + 1):
        idx_cluster = np.where(labels == i)[0]
        cluster_pcd = pcd_limpo.select_by_index(idx_cluster)
        
        aabb = cluster_pcd.get_axis_aligned_bounding_box()
        extents = aabb.get_extent() 
        volume = extents[0] * extents[1] * extents[2]
        altura = extents[2] 
        
        # Filtro heurístico (ajustado para deixar passar mesas e cadeiras)
        if (0.10 < altura < 2.0) and (0.01 < volume < 5.0) and (len(idx_cluster) > 30):
            print(f"Processando Cluster {i}: Altura={altura:.2f}m, Volume={volume:.2f}m³")
            
            # --- GERAÇÃO DA MESH ---
            # Alpha Shape cria a malha a partir da nuvem de pontos
            mesh_cluster = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cluster_pcd, alpha)
            
            # Limpeza da Mesh: remover triângulos duplicados ou não conectados
            mesh_cluster.remove_duplicated_vertices()
            mesh_cluster.remove_duplicated_triangles()
            
            # Adicionar esta parte à mesh final
            mesh_final += mesh_cluster

# =========================================================================
# PÓS-PROCESSAMENTO E ROTAÇÃO
# =========================================================================
if not mesh_final.is_empty():
    # 1. Calcular Normais (importante para sombras no RViz/Gazebo)
    mesh_final.compute_vertex_normals()

    # 2. Simplificação (Decimação) - Torna o ficheiro leve
    if len(mesh_final.triangles) > simplificacao_triangulos:
        print(f"Simplificando de {len(mesh_final.triangles)} para {simplificacao_triangulos} triângulos...")
        mesh_final = mesh_final.simplify_quadric_decimation(target_number_of_triangles=simplificacao_triangulos)

    # 4. GRAVAR O FICHEIRO STL
    o3d.io.write_triangle_mesh(nome_saida_stl, mesh_final)
    print(f"\n-> SUCESSO: STL gravado como '{nome_saida_stl}'")
else:
    print("\n-> ERRO: Nenhuma mesh foi gerada. Tenta aumentar o valor de 'alpha'.")
