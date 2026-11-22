#pragma once

#include <WindowItemFormatableLabel.hpp>
#include <WindowMenuSpin.hpp>

class MI_MMU_FRONT_PTFE_LENGTH : public WiSpin {
public:
    MI_MMU_FRONT_PTFE_LENGTH();

protected:
    virtual void OnClick() override;
    void Loop() override; ///< OneDrop3D (F41) : affiche la longueur que le MMU utilise vraiment

private:
    uint32_t warned = 0; ///< OneDrop3D : écriture refusée affichée (demandée << 16 | relue), 0 sans
    StringViewUtf8Parameters<16> label_params;
};
