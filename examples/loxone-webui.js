// Adds a "Loxone UDP template" box above the ESPHome web UI: a download link
// for GET /loxone/udp.xml (served by the gateway with its own IP and the
// configured UDP port) and the steps to use it. Included via
// web_server: js_include: in loxone-portal.yaml.
const de = (navigator.language || "").toLowerCase().startsWith("de");
const t = de
  ? {
      title: "Loxone: Virtueller UDP-Eingang",
      download: "Vorlage herunterladen (VIU_TWC_Gateway.xml)",
      steps: [
        "Unten unter „Loxone UDP Port“ einen Port eintragen (z. B. 7000) und „Loxone API Input“ leeren.",
        "Vorlage herunterladen – Absender-IP und Port sind schon eingetragen.",
        "Loxone Config: Virtuelle Eingänge → Virtueller UDP Eingang → Vorlage importieren.",
        "„TWC Vc“, „TWC Cac“, „TWC Cp“, „TWC Mr“ mit Vc, Cac, Cp, Mr des Wallbox-Bausteins verbinden; den API-Connector trennen.",
        "Ca und Tp des Bausteins bleiben am Virtuellen Ausgang (/loxone/set?ca=…, /loxone/set?tp=…).",
      ],
    }
  : {
      title: "Loxone: Virtual UDP Input",
      download: "Download template (VIU_TWC_Gateway.xml)",
      steps: [
        "Below, set “Loxone UDP Port” (e.g. 7000) and clear “Loxone API Input”.",
        "Download the template - sender IP and port are already filled in.",
        "Loxone Config: Virtual Inputs → Virtual UDP Input → import template.",
        "Connect “TWC Vc”, “TWC Cac”, “TWC Cp”, “TWC Mr” to the Wallbox block's Vc, Cac, Cp, Mr; disconnect the API connector.",
        "The block's Ca and Tp stay on the Virtual Output (/loxone/set?ca=…, /loxone/set?tp=…).",
      ],
    };

const box = document.createElement("details");
box.style.cssText =
  "margin:12px auto;max-width:960px;padding:10px 16px;border:1px solid #8884;" +
  "border-radius:8px;font-family:sans-serif;font-size:14px;line-height:1.5";
const summary = document.createElement("summary");
summary.textContent = t.title;
summary.style.cssText = "cursor:pointer;font-weight:600";
box.appendChild(summary);

const link = document.createElement("a");
link.href = "/loxone/udp.xml";
link.download = "VIU_TWC_Gateway.xml";
link.textContent = t.download;
link.style.cssText = "display:inline-block;margin:8px 0;font-weight:600";
box.appendChild(link);

const list = document.createElement("ol");
list.style.cssText = "margin:4px 0 0;padding-left:20px";
for (const step of t.steps) {
  const li = document.createElement("li");
  li.textContent = step;
  list.appendChild(li);
}
box.appendChild(list);

document.body.insertBefore(box, document.body.firstChild);
