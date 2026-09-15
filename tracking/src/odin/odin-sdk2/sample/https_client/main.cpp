#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>

#include "http_client.h"
#include "http_public.h"

static const std::string kBaseUrl = "https://192.168.1.251:60001";

/* ------------------------------------------------------------------ */
/*  progress bar helpers                                                */
/* ------------------------------------------------------------------ */
static void print_progress_bar(const char *label, uint64_t current, uint64_t total) {
  const int kBarWidth = 40;
  double ratio = (total > 0) ? (double)current / (double)total : 0.0;
  if (ratio > 1.0) ratio = 1.0;
  int filled = (int)(ratio * kBarWidth);

  printf("\r%s [", label);
  for (int i = 0; i < kBarWidth; i++) printf(i < filled ? "#" : "-");
  printf("] %3d%%  %lu / %lu bytes", (int)(ratio * 100), (unsigned long)current,
         (unsigned long)total);
  fflush(stdout);
  if (current >= total && total > 0) printf("\n");
}

static int on_download_progress(uint64_t downloaded, uint64_t total) {
  print_progress_bar("Download", downloaded, total);
  return 0;
}

static int on_upload_progress(uint64_t uploaded, uint64_t total) {
  print_progress_bar("Upload  ", uploaded, total);
  return 0;
}

/* ------------------------------------------------------------------ */
/*  build handle_opts with progress callbacks                           */
/* ------------------------------------------------------------------ */
static http::handle_opts make_client_opts() {
  http::handle_opts opts{};
  opts.progress_download_callback = on_download_progress;
  opts.progress_upload_callback = on_upload_progress;
  return opts;
}

/* ------------------------------------------------------------------ */
/*  usage                                                               */
/* ------------------------------------------------------------------ */
static void print_usage(const char *program) {
  std::cout << "Usage: " << program << " <command> [options]\n"
            << "Device: " << kBaseUrl << "\n"
            << "Commands:\n"
            << "  get_logs <output_dir>          - Download device logs\n"
            << "  get_calibration <output_dir>   - Download calibration file\n"
            << "  get_relocation_map <output_dir> - Download relocation map\n"
            << "  upload_firmware <file>          - Upload firmware and trigger OTA\n"
            << "  /* sign/verify moved to sign_tool & firmware_tool (V1.7.0) */\n";
}

/* ------------------------------------------------------------------ */
/*  commands                                                            */
/* ------------------------------------------------------------------ */
static int cmd_get_file(int argc, char *argv[], const std::string &command) {
  if (argc < 3) {
    std::cerr << "Error: " << command << " requires <output_dir>" << std::endl;
    return 1;
  }

  std::string output_dir = argv[2];
  http::Client client;
  client.set_handle_opts(make_client_opts());
  bool ok = false;

  if (command == "get_logs") {
    std::cout << "Getting device logs -> " << output_dir << std::endl;
    ok = client.download_device_logs(kBaseUrl, output_dir);
  } else if (command == "get_calibration") {
    std::cout << "Getting calibration file -> " << output_dir << std::endl;
    ok = client.download_calibration(kBaseUrl, output_dir);
  } else if (command == "get_relocation_map") {
    std::cout << "Getting relocation map -> " << output_dir << std::endl;
    ok = client.download_relocation_map(kBaseUrl, output_dir);
  }

  if (!ok) {
    std::cerr << "\nFailed: " << command << std::endl;
    return 1;
  }

  std::cout << "Download complete" << std::endl;
  return 0;
}

static int cmd_upload_firmware(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Error: upload_firmware requires <file>" << std::endl;
    return 1;
  }

  std::string file_path = argv[2];
  std::cout << "Uploading firmware: " << file_path << std::endl;

  http::Client client;
  client.set_handle_opts(make_client_opts());
  if (!client.upload_firmware_and_OTA(kBaseUrl, file_path)) {
    std::cerr << "\nUpload failed" << std::endl;
    return 1;
  }

  std::cout << "Upload complete" << std::endl;
  return 0;
}

/* sign/verify commands removed — use sign_tool & firmware_tool (V1.7.0+) */

/* ------------------------------------------------------------------ */
/*  main                                                                */
/* ------------------------------------------------------------------ */
int main(int argc, char *argv[]) {
  std::cout << "=== HTTPS Client (PC) ===" << std::endl;

  /* set mongoose log level */
  // http::log_set_level(http::kLogNone);

  if (argc < 2) {
    print_usage(argv[0]);
    return 1;
  }

  std::string command = argv[1];

  if (command == "get_logs" || command == "get_calibration" || command == "get_relocation_map") {
    return cmd_get_file(argc, argv, command);
  } else if (command == "upload_firmware") {
    return cmd_upload_firmware(argc, argv);
  } else if (command == "help" || command == "-h" || command == "--help") {
    print_usage(argv[0]);
    return 0;
  } else {
    std::cerr << "Unknown command: " << command << std::endl;
    print_usage(argv[0]);
    return 1;
  }
}
