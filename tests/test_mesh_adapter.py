from onshape_to_robot.mesh_adapter import GLBAdapter, STLAdapter, mesh_adapter_for


class TestAdapterDispatch:
    def test_stl_default(self):
        adapter = mesh_adapter_for("stl")
        assert isinstance(adapter, STLAdapter)
        assert adapter.extension == ".stl"

    def test_glb(self):
        adapter = mesh_adapter_for("glb")
        assert isinstance(adapter, GLBAdapter)
        assert adapter.extension == ".glb"

    def test_unknown_falls_back_to_stl(self):
        # Values are pre-validated by Config, so unknown here is just a safety net.
        adapter = mesh_adapter_for("unknown")
        assert isinstance(adapter, STLAdapter)
