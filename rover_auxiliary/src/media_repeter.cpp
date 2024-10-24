#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

void print_gstreamer_error(GError* error) {
    if (error) {
        g_printerr("GStreamer Error: %s\n", error->message);
        g_error_free(error);
    }
}

int main(int argc, char* argv[]) {
    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Enable debugging
    g_setenv("GST_DEBUG", "2", 1);  // Set debug level (0-9)

    // Create a new RTSP server
    GstRTSPServer* server = gst_rtsp_server_new();
    if (!server) {
        g_printerr("Failed to create RTSP server\n");
        return -1;
    }

    gst_rtsp_server_set_service(server, "8500");  // Set the RTSP port

    // Create a new mount point factory
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);
    if (!mounts) {
        g_printerr("Failed to get mount points\n");
        g_object_unref(server);
        return -1;
    }

    // Create a media factory to handle the RTSP stream
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
    if (!factory) {
        g_printerr("Failed to create media factory\n");
        g_object_unref(mounts);
        g_object_unref(server);
        return -1;
    }

    // Define the pipeline to pull the RTSP source and rebroadcast
    const gchar* pipeline_description = "rtspsrc location=rtsp://admin:admin@192.168.144.62:69 latency=0 buffer-mode=none ! "
                                        "rtph264depay ! rtph264pay name=pay0 pt=96";

    GError* error = nullptr;

    // Set the launch description
    gst_rtsp_media_factory_set_launch(factory, pipeline_description);

    // Set the factory to share the same pipeline with all clients
    gst_rtsp_media_factory_set_shared(factory, TRUE);

    // Attach the factory to a specific mount point (e.g., "/stream")
    gst_rtsp_mount_points_add_factory(mounts, "/stream", factory);

    // Unreference mounts after use
    g_object_unref(mounts);

    // Attach the server to the default main context
    if (gst_rtsp_server_attach(server, nullptr) == 0) {
        g_printerr("Failed to attach the server\n");
        g_object_unref(factory);
        g_object_unref(server);
        return -1;
    }

    // Start the main loop to handle incoming client connections
    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    if (!loop) {
        g_printerr("Failed to create main loop\n");
        g_object_unref(factory);
        g_object_unref(server);
        return -1;
    }

    g_printerr("RTSP server running, waiting for clients on rtsp://192.168.144.20:8554/stream...\n");
    g_main_loop_run(loop);
    g_printerr("Main loop exited\n");

    // Clean up resources
    g_object_unref(factory);
    g_object_unref(server);
    g_main_loop_unref(loop);

    return 0;
}
