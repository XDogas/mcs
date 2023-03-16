#ifndef ARTERY_MCOBJECT_H_
#define ARTERY_MCOBJECT_H_

#include <omnetpp/cobject.h>
#include <vanetza/asn1/mcm.hpp>
#include <memory>

namespace artery
{

class McObject : public omnetpp::cObject
{
    public:
        McObject(const McObject&) = default;
        McObject& operator=(const McObject&) = default;

        McObject(vanetza::asn1::Mcm&&);
        McObject& operator=(vanetza::asn1::Mcm&&);

        McObject(const vanetza::asn1::Mcm&);
        McObject& operator=(const vanetza::asn1::Mcm&);

        McObject(const std::shared_ptr<const vanetza::asn1::Mcm>&);
        McObject& operator=(const std::shared_ptr<const vanetza::asn1::Mcm>&);

        const vanetza::asn1::Mcm& asn1() const;

        std::shared_ptr<const vanetza::asn1::Mcm> shared_ptr() const;

        omnetpp::cObject* dup() const override;

    private:
        std::shared_ptr<const vanetza::asn1::Mcm> m_mcm_wrapper;
};

} // namespace artery

#endif /* ARTERY_MCOBJECT_H_ */
