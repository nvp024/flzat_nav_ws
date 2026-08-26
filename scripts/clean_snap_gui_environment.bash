# Sanitize GUI-library paths inherited from the Snap build of VS Code.
#
# A terminal opened inside Snap VS Code points GTK/GIO at the core20 runtime.
# Native Ubuntu 24.04 applications such as RViz and Gazebo must not load those
# older libraries. Keep normal desktop terminal environments unchanged.

if [[ "${SNAP_NAME:-}" == "code" || "${GTK_PATH:-}" == /snap/code/* ]]; then
  unset GDK_PIXBUF_MODULEDIR
  unset GDK_PIXBUF_MODULE_FILE
  unset GIO_MODULE_DIR
  unset GSETTINGS_SCHEMA_DIR
  unset GTK_EXE_PREFIX
  unset GTK_IM_MODULE_FILE
  unset GTK_MODULES
  unset GTK_PATH
  unset LOCPATH
fi
