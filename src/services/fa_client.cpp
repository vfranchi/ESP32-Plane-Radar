#include "services/fa_client.h"

#include <Arduino.h>
#include <Preferences.h>

#include <cstring>

namespace services::fa {
namespace {

// Same namespace as the radar's other settings, so the long-press credential wipe
// clears it with them. Handles are opened per operation, never held: ui::radar and
// follow mode open the same namespace and two open handles conflict.
constexpr char kPrefsNamespace[] = "planeradar";
constexpr char kPrefsKey[] = "fa_key";

char s_key[kKeyLen] = {};

}  // namespace

void loadKey() {
  s_key[0] = '\0';
  Preferences prefs;
  if (prefs.begin(kPrefsNamespace, true)) {
    prefs.getString(kPrefsKey, s_key, sizeof(s_key));
    prefs.end();
  }
  s_key[kKeyLen - 1] = '\0';
}

void saveKeyFromPortal(const char* text) {
  Preferences prefs;
  if (prefs.begin(kPrefsNamespace, false)) {
    if (text != nullptr && text[0] != '\0') {
      prefs.putString(kPrefsKey, text);
    } else {
      prefs.remove(kPrefsKey);
    }
    prefs.end();
  }
  loadKey();
  // The key itself never goes to the log: only whether there is one, and its length.
  Serial.printf("fa: key %s (%u chars)\n", s_key[0] != '\0' ? "set" : "cleared",
                static_cast<unsigned>(strlen(s_key)));
}

bool keyPresent() { return s_key[0] != '\0'; }

const char* apiKey() { return s_key; }

}  // namespace services::fa
