#include <mitsuba/render/mesh.h>
#include <mitsuba/render/bsdf.h>
#include <enoki/color.h>
#include <array>

// Blender Mesh format types for the exporter
NAMESPACE_BEGIN(blender)
    /// Smooth shading flag
    static const int ME_SMOOTH = 1 << 0;

    /// Triangle tesselation of the mesh, contains references to 3 MLoop and the "real" face
    struct MLoopTri {
        unsigned int tri[3];
        unsigned int poly;
    };

    struct MLoopUV {
        float uv[2];
        int flag;
    };

    struct MLoopCol {
        unsigned char r, g, b, a;
    };

    struct MLoop {
        /// Vertex index.
        unsigned int v;
        /// Edge index.
        unsigned int e;
    };

    /// Contains info about the face, like material ID and smooth shading flag
    struct MPoly {
        /// Offset into loop array and number of loops in the face
        int loopstart;
        int totloop;
        short mat_nr;
        char flag, _pad;
    };

    struct MVert {
        // Position
        float co[3];
        // Normal
        short no[3];
        char flag, bweight;
    };
NAMESPACE_END(blender)

NAMESPACE_BEGIN(mitsuba)

template <typename Float, typename Spectrum>
class BlenderMeshImpl final : public Mesh<Float, Spectrum> {
public:
    MTS_IMPORT_BASE(Mesh, m_name, m_to_world, m_vertex_count,
                    m_face_count, m_vertex_positions, m_vertex_normals,
                    m_vertex_texcoords, m_faces, m_disable_vertex_normals,
                    m_bsdf, add_attribute, set_children)
    MTS_IMPORT_TYPES(BSDF)

    using typename Base::FloatStorage;
    using typename Base::ScalarIndex;
    using typename Base::ScalarSize;
    using typename Base::InputFloat;
    using typename Base::InputVector2f;
    using ScalarIndex3 = std::array<ScalarIndex, 3>;

    BlenderMeshImpl(const Properties &props,
                    BSDF *bsdf,
                    const std::vector<std::array<InputFloat, 3>> &vertices,
                    const std::vector<std::array<InputFloat, 3>> &normals,
                    const std::vector<InputVector2f> &uvs,
                    const std::vector<std::vector<InputFloat>> &cols,
                    const std::vector<std::string> &col_names,
                    const std::vector<ScalarIndex3> &triangles,
                    ScalarIndex vertex_ctr,
                    bool shade_flat,
                    bool has_uvs,
                    bool has_cols) : Base(props) {

        m_vertex_count = vertex_ctr;
        m_disable_vertex_normals = shade_flat;

        m_bsdf = bsdf;
        m_face_count = (ScalarSize) triangles.size();
        m_faces = ek::load_unaligned<DynamicBuffer<UInt32>>(triangles.data(), m_face_count * 3);

        m_vertex_positions = ek::load_unaligned<FloatStorage>(vertices.data(), m_vertex_count * 3);
        if (!m_disable_vertex_normals)
            m_vertex_normals = ek::load_unaligned<FloatStorage>(normals.data(), m_vertex_count * 3);

        if (has_uvs)
            m_vertex_texcoords = ek::load_unaligned<FloatStorage>(uvs.data(), m_vertex_count * 2);

        if (has_cols) {
            for (size_t p = 0; p < cols.size(); p++)
                add_attribute(col_names[p], 3, cols[p]);
        }

        set_children();

    }

    MTS_DECLARE_CLASS()

};

MTS_IMPLEMENT_CLASS_VARIANT(BlenderMeshImpl, Mesh)

/**

Blender mesh loader
----------------------------------------------------------

This plugins converts a Blender Mesh to mitsuba's mesh layout. It is used in the Blender exporter Add-on.
It expects as input pointers to Blender mesh data structures, see GeometryExporter.save_mesh
in the mitsuba2-blender addon for an example.
 */

template <typename Float, typename Spectrum>
class BlenderMesh final : public Mesh<Float, Spectrum> {
public:
    MTS_IMPORT_BASE(Mesh, m_name, m_bbox, m_to_world, m_vertex_count,
                    m_face_count, m_vertex_positions, m_vertex_normals,
                    m_vertex_texcoords, m_faces, m_disable_vertex_normals, add_attribute, set_children)
    MTS_IMPORT_TYPES(BSDF)

    using typename Base::MeshAttributeType;
    using typename Base::ScalarSize;
    using typename Base::ScalarIndex;
    using typename Base::InputFloat;
    using typename Base::InputPoint3f;
    using typename Base::InputVector2f;
    using typename Base::InputVector3f;
    using typename Base::InputNormal3f;

    using ScalarIndex3 = std::array<ScalarIndex, 3>;

    /**
    * This constructor created a Mesh object from the part of a blender mesh assigned to a certain material.
    * This allows exporting meshes with multiple materials.
    * This method is inspired by LuxCoreRender (https://github.com/LuxCoreRender/LuxCore/blob/master/src/luxcore/pyluxcoreforblender.cpp)
    * \param props Contains counters and pointers to blender's data structures
    */
    BlenderMesh(const Properties &props) : Base(props) {

        auto fail = [&](const char *descr, auto... args) {
            Throw(("Error while loading Blender mesh \"%s\": " + std::string(descr))
                    .c_str(),
                m_name, args...);
        };

        std::vector<std::string> necessary_fields = {"name", "vert_count", "loop_tri_count", "loops", "loop_tris", "polys", "verts"};
        for (auto &s : necessary_fields) {
            if (!props.has_property(s))
                fail("Missing property \"%s\"!", s);
        }
        m_props = props;
        m_name     = props.string("name");
        size_t vertex_count(props.int_("vert_count"));
        size_t loop_tri_count(props.int_("loop_tri_count"));
        const blender::MLoop *loops =
            reinterpret_cast<const blender::MLoop *>(props.long_("loops"));
        const blender::MLoopTri *tri_loops =
            reinterpret_cast<const blender::MLoopTri *>(props.long_("loop_tris"));
        const blender::MPoly *polygons =
            reinterpret_cast<const blender::MPoly *>(props.long_("polys"));
        const blender::MVert *verts =
            reinterpret_cast<const blender::MVert *>(props.long_("verts"));

        m_has_cols = false;
        std::vector<std::pair<std::string, const blender::MLoopCol *>> cols;
        for (std::string &s : props.property_names()){
            if (s.rfind("vertex_", 0) == 0){
                cols.push_back({s, reinterpret_cast<const blender::MLoopCol *>(props.long_(s))});
                m_col_names.push_back(s);
                m_has_cols = true;
            }
        }

        m_has_uvs = props.has_property("uvs");
        const blender::MLoopUV *uvs = nullptr;
        if (m_has_uvs)
            uvs = reinterpret_cast<const blender::MLoopUV *>(props.long_("uvs"));
        else
            Log(Warn, "Mesh %s has no texture coordinates!", m_name);

        // Find all the material ids and their associated materials.
        // TODO: same with emitters
        m_mat_count = 0;
        std::string mat_id = "mat_0";
        std::vector<std::pair<std::string, ref<Object>>> objects = props.objects(false);
        while (props.has_property(mat_id)) {
            m_mat_count++;
            for (auto &[name, obj] : objects) {
                if (name == mat_id)
                    m_bsdfs.push_back(dynamic_cast<BSDF *>(obj.get()));
            }
            mat_id = "mat_" + std::to_string(m_mat_count);
        }

        // Determine whether the object is globally smooth or flat shaded and set the flag accordingly
        // Blender meshes can be partially smooth AND flat (e.g. with edge split modifier)
        // In this case, flat face vertices will be duplicated.
        m_shade_flat.resize(m_mat_count, true);
        u_short flat_parts= m_mat_count;
        for (size_t tri_loop_id = 0; tri_loop_id < loop_tri_count; tri_loop_id++) {
            const blender::MLoopTri &tri_loop = tri_loops[tri_loop_id];
            const blender::MPoly &face        = polygons[tri_loop.poly];
            if ((blender::ME_SMOOTH & face.flag) && m_shade_flat[face.mat_nr]) {
                // If at least one face is smooth shaded, we need to disable face normals
                // and duplicate the (potential) flat shaded face vertices.
                m_shade_flat[face.mat_nr] = false;
                flat_parts--;
                if (flat_parts==0) // Stop if we have determined all parts of the mesh are not globally flat
                    break;
            }
        }

        tmp_vertices.reserve(m_mat_count);
        tmp_normals.reserve(m_mat_count);
        tmp_triangles.reserve(m_mat_count);
        tmp_uvs.reserve(m_mat_count);
        tmp_cols.reserve(m_mat_count);
        for (u_short i=0; i<m_mat_count; i++) {
            tmp_vertices[i].reserve(vertex_count);
            if (!m_shade_flat[i])
                tmp_normals[i].reserve(vertex_count);
            tmp_triangles[i].reserve(loop_tri_count);
            if (m_has_uvs)
                tmp_uvs[i].reserve(vertex_count);
            if (m_has_cols) {
                tmp_cols[i].reserve(cols.size());
                for (size_t p = 0; p < cols.size(); p++) {
                    tmp_cols[i].push_back(std::vector<InputFloat>());
                    tmp_cols[i][p].reserve(3 * vertex_count);
                }
            }
        }
        m_vertex_ctr.reserve(m_mat_count);

        // Hash map key to define a unique vertex
        struct Key {
            InputNormal3f normal;
            bool smooth{ false };
            size_t poly{ 0 }; // store the polygon face for flat shading, since
                              // comparing normals is ambiguous due to numerical
                              // precision
            InputVector2f uv{ 0.0f, 0.0f };
            bool operator==(const Key &other) const {
                return (smooth ? normal == other.normal : poly == other.poly) &&
                       uv == other.uv;
            }
            bool operator!=(const Key &other) const {
                return (smooth ? normal != other.normal : poly != other.poly) ||
                       uv != other.uv;
            }
        };
        // Hash map entry:
        struct VertexBinding {
            Key key;
            // Index of the vertex in the vertex array
            ScalarIndex value{ 0 };
            VertexBinding *next{ nullptr };
            bool is_init{ false };
        };

        // Hash Map to avoid adding duplicate vertices
        std::vector<std::vector<VertexBinding>> vertex_maps(m_mat_count);
        for (u_short i=0; i<m_mat_count; i++) {
            vertex_maps[i].reserve(vertex_count);
        }

        size_t duplicates_ctr = 0;
        for (size_t tri_loop_id = 0; tri_loop_id < loop_tri_count; tri_loop_id++) {
            const blender::MLoopTri &tri_loop = tri_loops[tri_loop_id];
            const blender::MPoly &face        = polygons[tri_loop.poly];

            ScalarIndex3 triangle;

            const blender::MVert &v0 = verts[loops[tri_loop.tri[0]].v];
            const blender::MVert &v1 = verts[loops[tri_loop.tri[1]].v];
            const blender::MVert &v2 = verts[loops[tri_loop.tri[2]].v];

            ek::Array<InputPoint3f, 3> face_points;
            face_points[0] = InputPoint3f(v0.co[0], v0.co[1], v0.co[2]);
            face_points[1] = InputPoint3f(v1.co[0], v1.co[1], v1.co[2]);
            face_points[2] = InputPoint3f(v2.co[0], v2.co[1], v2.co[2]);

            InputNormal3f normal(0.f);
            if (!(blender::ME_SMOOTH & face.flag) && !m_disable_vertex_normals) {
                // Flat shading, use per face normals (only if the mesh is not globally flat)
                const InputVector3f e1 = face_points[1] - face_points[0];
                const InputVector3f e2 = face_points[2] - face_points[0];
                normal = m_to_world.transform_affine(ek::cross(e1, e2));
                if(unlikely(ek::all(ek::eq(normal, 0.f))))
                    continue; // Degenerate triangle, ignore it
                else
                    normal = ek::normalize(normal);
            }

            InputFloat color_factor = ek::rcp(255.f);

            for (int i = 0; i < 3; i++) {
                const size_t loop_index = tri_loop.tri[i];
                const size_t vert_index = loops[loop_index].v;
                if (unlikely((vert_index >= vertex_count)))
                    fail("reference to invalid vertex %i!", vert_index);

                const blender::MVert &vert = verts[vert_index];

                Key vert_key;
                if (blender::ME_SMOOTH & face.flag || m_disable_vertex_normals) {
                    // Store per vertex normals if the face is smooth or if the mesh is globally flat
                    normal = m_to_world.transform_affine(InputNormal3f(vert.no[0], vert.no[1], vert.no[2]));
                    if(unlikely(ek::all(ek::eq(normal, 0.f))))
                        fail("Mesh has invalid normals!");
                    else
                        normal = ek::normalize(normal);
                    vert_key.smooth = true;
                } else {
                    // vert_key.smooth = false (default), flat shading
                    // Store the referenced polygon (face)
                    vert_key.poly = tri_loop.poly;
                }

                vert_key.normal = normal;

                if (m_has_uvs) {
                    const blender::MLoopUV &loop_uv = uvs[loop_index];
                    const InputVector2f uv(loop_uv.uv[0], 1.0f - loop_uv.uv[1]);
                    vert_key.uv = uv;
                }

                // Vertex index in the blender mesh is the map index
                VertexBinding *map_entry = &vertex_maps[face.mat_nr][vert_index];
                while (vert_key != map_entry->key && map_entry->next != nullptr)
                    map_entry = map_entry->next;

                if (map_entry->is_init && map_entry->key == vert_key) {
                    triangle[i] = map_entry->value;
                    duplicates_ctr++;
                } else {
                    if (map_entry->is_init) {
                        // Add a new entry
                        map_entry->next = new VertexBinding();
                        map_entry       = map_entry->next;
                    }
                    ScalarSize vert_id = m_vertex_ctr[face.mat_nr]++;
                    map_entry->key     = vert_key;
                    map_entry->value   = vert_id;
                    map_entry->is_init = true;
                    // Add stuff to the temporary buffers
                    InputPoint3f pt = m_to_world.transform_affine(face_points[i]);
                    tmp_vertices[face.mat_nr].push_back({pt.x(), pt.y(), pt.z()});
                    if (!m_disable_vertex_normals)
                        tmp_normals[face.mat_nr].push_back({normal.x(), normal.y(), normal.z()});
                    if (m_has_uvs)
                        tmp_uvs[face.mat_nr].push_back(vert_key.uv);
                    if (m_has_cols) {
                        for (size_t p = 0; p < cols.size(); p++) {
                            const blender::MLoopCol &loop_col = cols[p].second[loop_index];
                            // Blender stores vertex colors in sRGB space
                            tmp_cols[face.mat_nr][p].push_back(ek::srgb_to_linear(loop_col.r * color_factor));
                            tmp_cols[face.mat_nr][p].push_back(ek::srgb_to_linear(loop_col.g * color_factor));
                            tmp_cols[face.mat_nr][p].push_back(ek::srgb_to_linear(loop_col.b * color_factor));
                        }
                    }
                    triangle[i] = vert_id;
                }
            }
            tmp_triangles[face.mat_nr].push_back(triangle);
        }
        Log(Info, "%s: Removed %i duplicates", m_name, duplicates_ctr);

    }

    // Split this object into multiple sub meshes
    std::vector<ref<Object>> expand() const override {
        std::vector<ref<Object>> meshes;
        for (u_short i=0; i<m_mat_count; i++) {
            if (m_vertex_ctr[i] > 0) {
                meshes.push_back(ref<Object>(new BlenderMeshImpl<Float, Spectrum>(m_props,
                                                                m_bsdfs[i],
                                                                tmp_vertices[i],
                                                                tmp_normals[i],
                                                                tmp_uvs[i],
                                                                tmp_cols[i],
                                                                m_col_names,
                                                                tmp_triangles[i],
                                                                m_vertex_ctr[i],
                                                                m_shade_flat[i],
                                                                m_has_uvs,
                                                                m_has_cols)));
            }
        }
        return meshes;
    }

    MTS_DECLARE_CLASS()

private:
    u_short m_mat_count;
    std::vector<BSDF *> m_bsdfs;
    Properties m_props;
    // Buffers for storing vertices, normals, etc.
    std::vector<std::vector<std::array<InputFloat, 3>>> tmp_vertices; // Store as vector for alignement issues
    std::vector<std::vector<std::array<InputFloat, 3>>> tmp_normals; // Same here
    std::vector<std::vector<InputVector2f>> tmp_uvs;
    std::vector<std::vector<std::vector<InputFloat>>> tmp_cols; // And same here
    std::vector<std::vector<ScalarIndex3>> tmp_triangles;
    std::vector<ScalarIndex> m_vertex_ctr;
    std::vector<std::string> m_col_names;
    // Smooth/flat shading per sub-mesh
    std::vector<bool> m_shade_flat;
    bool m_has_cols;
    bool m_has_uvs;
};

MTS_IMPLEMENT_CLASS_VARIANT(BlenderMesh, Mesh)
MTS_EXPORT_PLUGIN(BlenderMesh, "Blender Mesh")


NAMESPACE_END(mitsuba)
