import os
import re

src_model = "osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim"
dst_model = "osim_files/HYOID_HeadNeckNeural_Model_Adaptive.osim"

with open(src_model, "r", encoding="utf-8") as f:
    xml = f.read()

# Replace <HeadNeckNeuralController with <HeadNeckNeuralControllerAdaptive
xml = xml.replace("<HeadNeckNeuralController name=\"VCR_CCR_Controller\">", 
                  "<HeadNeckNeuralControllerAdaptive name=\"VCR_CCR_Controller\">")
xml = xml.replace("</HeadNeckNeuralController>", "</HeadNeckNeuralControllerAdaptive>")

# Also set the default properties inside VCR_CCR_Controller
# delay_smith_proprio and delay_afferent_proprio
if "<delay_smith_proprio>" not in xml:
    xml = xml.replace("</HeadNeckNeuralControllerAdaptive>", 
                      "    <use_smith_predictor>true</use_smith_predictor>\n"
                      "    <delay_smith_proprio>0.007</delay_smith_proprio>\n"
                      "    <delay_afferent_proprio>0.007</delay_afferent_proprio>\n"
                      "    <enable_adaptation>true</enable_adaptation>\n"
                      "    <adaptation_rate_sc>0.15</adaptation_rate_sc>\n"
                      "    <adaptation_rate_ton>0.15</adaptation_rate_ton>\n"
                      "    <adaptation_rate_phas>0.40</adaptation_rate_phas>\n"
                      "    <adaptation_rate_ccr>0.20</adaptation_rate_ccr>\n"
                      "    <sigma_leakage>0.02</sigma_leakage>\n"
                      "    <adaptation_start_time>2.5</adaptation_start_time>\n"
                      "    <multisine_scale>0.142887</multisine_scale>\n"
                      "</HeadNeckNeuralControllerAdaptive>")

with open(dst_model, "w", encoding="utf-8") as f:
    f.write(xml)

print(f"Created {dst_model} with HeadNeckNeuralControllerAdaptive tag.")

