#[doc = "Reader of register CALM1"]
pub type R = crate::R<u32, super::CALM1>;
#[doc = "Writer for register CALM1"]
pub type W = crate::W<u32, super::CALM1>;
#[doc = "Register CALM1 `reset()`'s with value 0"]
impl crate::ResetValue for super::CALM1 {
    type Type = u32;
    #[inline(always)]
    fn reset_value() -> Self::Type {
        0
    }
}
#[doc = "Reader of field `DOM`"]
pub type DOM_R = crate::R<u8, u8>;
#[doc = "Write proxy for field `DOM`"]
pub struct DOM_W<'a> {
    w: &'a mut W,
}
impl<'a> DOM_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits = (self.w.bits & !0x1f) | ((value as u32) & 0x1f);
        self.w
    }
}
impl R {
    #[doc = "Bits 0:4 - Day of Month"]
    #[inline(always)]
    pub fn dom(&self) -> DOM_R {
        DOM_R::new((self.bits & 0x1f) as u8)
    }
}
impl W {
    #[doc = "Bits 0:4 - Day of Month"]
    #[inline(always)]
    pub fn dom(&mut self) -> DOM_W {
        DOM_W { w: self }
    }
}
